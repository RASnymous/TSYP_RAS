"""
lmb2.py - the Living Map beacon protocol (LMB2) codec, bundled with the Writer
robot package so the ROS nodes need nothing else installed.

This is an unmodified copy of beacon_net/python/beaconnet/proto.py (the file
the beacon firmware is cross-checked against). If you change the protocol,
change it there and copy it here.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math
import struct
from dataclasses import dataclass, field, replace
from typing import Iterable, List, Optional, Sequence, Tuple, Union

# ---------------------------------------------------------------- constants
VERSION = 2
TYPE_RECORD = 1
TYPE_DIGEST = 2
VER_TYPE = (VERSION << 4) | TYPE_RECORD      # 0x21
VER_DIGEST = (VERSION << 4) | TYPE_DIGEST    # 0x22
HDR_LEN, REC_LEN, MAC_LEN = 6, 30, 8
FRAME_LEN = HDR_LEN + REC_LEN + MAC_LEN      # 44
KEY_LEN = 16
ID_NONE = 0xFFFF
ERR_UNKNOWN = 255
MAX_HOP_LIMIT = 15
HOPS_UNSCOPED = 15
DIGEST_MAX_ENTRIES = 60
DIGEST_PAGE_MASK = 0x7F
DIGEST_FINAL = 0x80
DOMAIN_TAG = b"LMB2"
DEMO_KEY = b"LivingMap-DEMO!!"               # demo/simulation only - never in the field

# record flags (signed)
RF_HAS_NEXT = 0x01
RF_HAS_BEARING = 0x02
RF_POS_GNSS = 0x04
RF_RETRACTED = 0x08
RF_STALE = 0x10
# header flags (unsigned)
HF_ORIGIN_SILENT = 0x01

KINDS = ["WAYPOINT", "VICTIM", "GAS", "RADIATION", "THERMAL", "OBSTRUCTION", "STRUCTURAL", "EXIT",
         "PHOSPHATE", "GOLD", "GEMSTONE", "SEARCHED"]
KIND = {name: i for i, name in enumerate(KINDS)}
KIND_ALIASES = {"FIRE": 4, "HEAT": 4, "TRAIL": 0, "RUBBLE": 5, "BLOCKED": 5, "ROOF": 6, "COLLAPSE": 6,
                "ORE": 8, "SEAM": 8, "GEM": 10, "CLEAR": 11, "NO_VICTIM": 11}
# adaptive aging defaults, seconds (same tables as beacon_proto.c)
HALF_LIFE_S = [43200, 3600, 600, 21600, 1200, 172800, 86400, 172800, 259200, 259200, 259200, 7200]
TTL_S = [86400, 7200, 3600, 43200, 3600, 604800, 259200, 604800, 604800, 604800, 604800, 43200]
WEIGHT = [1.0, 3.0, 2.5, 2.5, 2.0, 1.5, 2.0, 1.5, 0.8, 0.8, 0.8, 1.2]

OK, ERR_LEN, ERR_VERSION, ERR_NET, ERR_MAC, ERR_FIELD = 0, -1, -2, -3, -4, -5
STATUS = {OK: "ok", ERR_LEN: "bad_length", ERR_VERSION: "bad_version", ERR_NET: "foreign_network",
          ERR_MAC: "bad_mac", ERR_FIELD: "bad_field"}


class ProtoError(ValueError):
    """Frame rejected. .status is the C status code, str() its name."""

    def __init__(self, status: int):
        super().__init__(STATUS.get(status, "unknown"))
        self.status = status


# ---------------------------------------------------------------- data model
@dataclass
class Record:
    origin_id: int
    seq: int = 1
    kind: int = 0
    flags: int = 0
    lat_e7: int = 0
    lon_e7: int = 0
    err_dm: int = ERR_UNKNOWN
    severity: int = 0
    confidence: int = 255
    next_bearing: int = 0
    next_id: int = ID_NONE
    next_dist_dm: int = 0
    timestamp: int = 0
    ttl_10s: int = 1
    half_life_10s: int = 1


@dataclass
class Frame:
    rec: Record
    net_id: int = 0x2A
    relay_id: int = 0
    hops: int = 0
    hop_limit: int = HOPS_UNSCOPED
    hdr_flags: int = 0


# ---------------------------------------------------------------- unit helpers
def _round_half_up(x: float) -> int:
    """C: (int)(x + 0.5) for x >= 0 (truncation toward zero)."""
    return int(x + 0.5)


def deg_to_e7(d: float) -> int:
    return int(d * 1e7 + 0.5) if d >= 0 else int(d * 1e7 - 0.5)


def e7_to_deg(e7: int) -> float:
    return e7 / 1e7


def bearing_to_u8(deg: float) -> int:
    d = math.fmod(deg, 360.0)
    if d < 0:
        d += 360.0
    return int(d * 256.0 / 360.0 + 0.5) & 0xFF


def u8_to_bearing(b: int) -> float:
    return b * 360.0 / 256.0


def seconds_to_10s(s: int) -> int:
    v = (int(s) + 5) // 10
    return min(max(v, 1), 0xFFFF)


def kind_from_name(name: str) -> int:
    n = name.upper()
    if n in KIND:
        return KIND[n]
    if n in KIND_ALIASES:
        return KIND_ALIASES[n]
    return -1


def kind_name(k: int) -> str:
    return KINDS[k] if 0 <= k < len(KINDS) else "UNKNOWN"


def default_half_life_s(k: int) -> int:
    return HALF_LIFE_S[k] if 0 <= k < len(KINDS) else 3600


def default_ttl_s(k: int) -> int:
    return TTL_S[k] if 0 <= k < len(KINDS) else 7200


# ---------------------------------------------------------------- aging
def effective_confidence(r: Record, now: int) -> float:
    """c0 * 2^(-age / half_life): gas fades in minutes, an obstruction in days."""
    hl = r.half_life_10s * 10.0
    if hl <= 0:
        return 0.0
    age = max(0, now - r.timestamp)
    return (r.confidence / 255.0) * 2.0 ** (-age / hl)


def is_expired(r: Record, now: int, min_conf: float = 0.05) -> bool:
    age = max(0, now - r.timestamp)
    return age > r.ttl_10s * 10 or effective_confidence(r, now) < min_conf


# ---------------------------------------------------------------- validation
def validate(r: Record) -> None:
    if not (-900000000 <= r.lat_e7 <= 900000000 and -1800000000 <= r.lon_e7 <= 1800000000):
        raise ProtoError(ERR_FIELD)
    if r.ttl_10s == 0 or r.half_life_10s == 0 or r.origin_id == ID_NONE:
        raise ProtoError(ERR_FIELD)
    if (r.flags & RF_HAS_NEXT) and r.next_id == ID_NONE:
        raise ProtoError(ERR_FIELD)


# ---------------------------------------------------------------- codec
_REC = struct.Struct("<HHBBiiBBBBHHIHH")   # 30 bytes, little-endian
assert _REC.size == REC_LEN


def pack_record(r: Record) -> bytes:
    return _REC.pack(r.origin_id, r.seq, r.kind, r.flags, r.lat_e7, r.lon_e7, r.err_dm, r.severity,
                     r.confidence, r.next_bearing, r.next_id, r.next_dist_dm, r.timestamp, r.ttl_10s,
                     r.half_life_10s)


def unpack_record(b: bytes) -> Record:
    return Record(*_REC.unpack(b))


def _hmac8(key: bytes, msg: bytes) -> bytes:
    return hmac.new(key, DOMAIN_TAG + msg, hashlib.sha256).digest()[:MAC_LEN]


def record_mac(key: bytes, ver_type: int, net_id: int, rec_bytes: bytes) -> bytes:
    """MAC = HMAC-SHA256(key, "LMB2" | ver_type | net_id | record30)[:8]"""
    return _hmac8(key, bytes([ver_type, net_id]) + rec_bytes)


def encode(f: Frame, key: bytes) -> bytes:
    if len(key) != KEY_LEN:
        raise ValueError("key must be 16 bytes")
    validate(f.rec)
    if not (0 <= f.hops <= MAX_HOP_LIMIT and 0 <= f.hop_limit <= MAX_HOP_LIMIT and f.hops <= f.hop_limit):
        raise ProtoError(ERR_FIELD)
    rec = pack_record(f.rec)
    hdr = struct.pack("<BBHBB", VER_TYPE, f.net_id, f.relay_id, (f.hops << 4) | f.hop_limit, f.hdr_flags)
    return hdr + rec + record_mac(key, VER_TYPE, f.net_id, rec)


def decode(buf: bytes, key: bytes, expect_net: int) -> Frame:
    """Authenticate + parse a 44-byte record frame. Raises ProtoError."""
    if len(buf) != FRAME_LEN:
        raise ProtoError(ERR_LEN)
    if buf[0] != VER_TYPE:
        raise ProtoError(ERR_VERSION)
    if buf[1] != expect_net:
        raise ProtoError(ERR_NET)
    rec_b = bytes(buf[HDR_LEN:HDR_LEN + REC_LEN])
    if not hmac.compare_digest(record_mac(key, buf[0], buf[1], rec_b), bytes(buf[HDR_LEN + REC_LEN:])):
        raise ProtoError(ERR_MAC)
    _, net, relay, hb, hflags = struct.unpack("<BBHBB", bytes(buf[:HDR_LEN]))
    f = Frame(rec=unpack_record(rec_b), net_id=net, relay_id=relay, hops=hb >> 4, hop_limit=hb & 0x0F,
              hdr_flags=hflags)
    if f.hops > f.hop_limit:
        raise ProtoError(ERR_FIELD)
    validate(f.rec)
    return f


def encode_digest(net_id: int, sender: int, entries: Sequence[Tuple[int, int]], key: bytes,
                  page: int = 0, final: bool = True) -> bytes:
    """Digest (pull) frame: every (origin, seq) the sender holds, sorted by origin."""
    if len(entries) > DIGEST_MAX_ENTRIES:
        raise ProtoError(ERR_FIELD)
    body = struct.pack("<BBHBB", VER_DIGEST, net_id, sender, len(entries),
                       (page & DIGEST_PAGE_MASK) | (DIGEST_FINAL if final else 0))
    body += b"".join(struct.pack("<HH", o, s) for o, s in entries)
    return body + _hmac8(key, body)


def decode_digest(buf: bytes, key: bytes, expect_net: int):
    """-> (sender, page, final, [(origin, seq), ...]). Raises ProtoError."""
    if len(buf) < 6 + MAC_LEN:
        raise ProtoError(ERR_LEN)
    if buf[0] != VER_DIGEST:
        raise ProtoError(ERR_VERSION)
    if buf[1] != expect_net:
        raise ProtoError(ERR_NET)
    n = buf[4]
    if n > DIGEST_MAX_ENTRIES or len(buf) != 6 + 4 * n + MAC_LEN:
        raise ProtoError(ERR_LEN)
    body = bytes(buf[:6 + 4 * n])
    if not hmac.compare_digest(_hmac8(key, body), bytes(buf[6 + 4 * n:])):
        raise ProtoError(ERR_MAC)
    sender = struct.unpack_from("<H", body, 2)[0]
    entries = [struct.unpack_from("<HH", body, 6 + 4 * i) for i in range(n)]
    return sender, buf[5] & DIGEST_PAGE_MASK, bool(buf[5] & DIGEST_FINAL), entries


# ---------------------------------------------------------------- LoRa airtime
def lora_airtime_ms(payload_len: int, sf: int = 9, bw_hz: int = 125000, cr_denom: int = 5,
                    preamble: int = 8, crc: bool = True, implicit: bool = False) -> int:
    """Semtech AN1200.13 time-on-air, ms rounded up (LDRO auto when Tsym > 16 ms)."""
    tsym = (1 << sf) / bw_hz * 1000.0
    de = 1 if tsym > 16.0 else 0
    num = 8.0 * payload_len - 4.0 * sf + 28 + 16 * (1 if crc else 0) - 20 * (1 if implicit else 0)
    n = math.ceil(num / (4.0 * (sf - 2 * de))) * ((cr_denom - 4) + 4)
    return int(math.ceil((preamble + 4.25 + 8 + max(n, 0)) * tsym))


# ---------------------------------------------------------------- JSON
def record_from_json(obj: Union[str, dict], now: int = 0) -> Record:
    """Mission-log JSON -> Record, same rules and defaults as bp_json.c."""
    d = json.loads(obj) if isinstance(obj, str) else obj
    if not isinstance(d, dict):
        raise ProtoError(ERR_FIELD)
    gps = d.get("gps") if isinstance(d.get("gps"), dict) else {}

    def num(v, lo, hi, whole=True):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ProtoError(ERR_FIELD)
        if not (lo <= v <= hi) or (whole and v != math.floor(v)):
            raise ProtoError(ERR_FIELD)
        return v

    bid = d.get("beacon_id", d.get("id"))
    lat = gps.get("lat", d.get("lat"))
    lon = gps.get("lon", d.get("lon"))
    if bid is None or lat is None or lon is None:
        raise ProtoError(ERR_FIELD)
    r = Record(origin_id=int(num(bid, 0, 65534)))
    r.seq = int(num(d["seq"], 0, 65535)) if "seq" in d else 1
    k = d.get("kind", d.get("type", "WAYPOINT"))
    if isinstance(k, str):
        r.kind = kind_from_name(k)
        if r.kind < 0:
            raise ProtoError(ERR_FIELD)
    else:
        r.kind = int(num(k, 0, len(KINDS) - 1))
    num(lat, -90.0, 90.0, whole=False)
    num(lon, -180.0, 180.0, whole=False)
    r.lat_e7, r.lon_e7 = deg_to_e7(lat), deg_to_e7(lon)
    err = gps.get("err_m", d.get("err_m"))
    if err is None:
        r.err_dm = ERR_UNKNOWN
    else:
        num(err, 0.0, math.inf, whole=False)
        r.err_dm = 254 if err >= 25.4 else _round_half_up(err * 10.0)
    r.severity = int(num(d["severity"], 0, 255)) if "severity" in d else 0
    r.confidence = _round_half_up(num(d["confidence"], 0.0, 1.0, whole=False) * 255.0) \
        if "confidence" in d else 255
    src = gps.get("src")
    if isinstance(src, str) and src.lower() in ("gnss", "gps"):
        r.flags |= RF_POS_GNSS
    if d.get("retracted") is True:
        r.flags |= RF_RETRACTED
    if d.get("stale") is True:
        r.flags |= RF_STALE
    nxt = d.get("next")
    if isinstance(nxt, dict) and "id" in nxt:
        r.flags |= RF_HAS_NEXT
        r.next_id = int(num(nxt["id"], 0, 65534))
        if "dist_m" in nxt:
            dist = num(nxt["dist_m"], 0.0, math.inf, whole=False)
            r.next_dist_dm = 0xFFFF if dist >= 6553.5 else _round_half_up(dist * 10.0)
        if "bearing_deg" in nxt:
            r.flags |= RF_HAS_BEARING
            r.next_bearing = bearing_to_u8(num(nxt["bearing_deg"], -math.inf, math.inf, whole=False))
    r.timestamp = int(num(d["ts"], 0, 4294967295)) if "ts" in d else int(now)
    ttl = num(d["ttl_s"], 1e-9, 655350.0, whole=False) if "ttl_s" in d else default_ttl_s(r.kind)
    hl = num(d["half_life_s"], 1e-9, 655350.0, whole=False) if "half_life_s" in d else default_half_life_s(r.kind)
    r.ttl_10s = seconds_to_10s(int(ttl))
    r.half_life_10s = seconds_to_10s(int(hl))
    validate(r)
    return r


def record_to_dict(r: Record, now: Optional[int] = None) -> dict:
    """Record -> the mission-log JSON shape (plus eff_conf when `now` is given)."""
    d = {
        "beacon_id": r.origin_id,
        "seq": r.seq,
        "kind": kind_name(r.kind),
        "gps": {"lat": round(e7_to_deg(r.lat_e7), 7), "lon": round(e7_to_deg(r.lon_e7), 7),
                "err_m": None if r.err_dm == ERR_UNKNOWN else r.err_dm / 10.0,
                "src": "gnss" if r.flags & RF_POS_GNSS else "slam"},
        "severity": r.severity,
        "confidence": round(r.confidence / 255.0, 2),
        "next": None,
        "ts": r.timestamp,
        "ttl_s": r.ttl_10s * 10,
        "half_life_s": r.half_life_10s * 10,
        "retracted": bool(r.flags & RF_RETRACTED),
        "stale": bool(r.flags & RF_STALE),
    }
    if r.flags & RF_HAS_NEXT:
        d["next"] = {"id": r.next_id, "dist_m": r.next_dist_dm / 10.0}
        if r.flags & RF_HAS_BEARING:
            d["next"]["bearing_deg"] = round(u8_to_bearing(r.next_bearing), 1)
    if now is not None:
        d["eff_conf"] = round(effective_confidence(r, now), 3)
        d["age_s"] = max(0, now - r.timestamp)
    return d


def key_from_hex(hexstr: str) -> bytes:
    k = bytes.fromhex(hexstr)
    if len(k) != KEY_LEN:
        raise ValueError("key must be 32 hex characters")
    return k


__all__ = [n for n in dir() if not n.startswith("_")]
