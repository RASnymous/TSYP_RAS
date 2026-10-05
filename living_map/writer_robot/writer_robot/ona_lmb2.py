# ---------------------------------------------------------------------------
# COPY of living_map/ona/ona/lmb2.py (the Outside Network Area's frame
# codec), so that the robots can talk to the ONA without the ONA installed.
# tools/test_ona_link.py checks that both copies are identical.
# ---------------------------------------------------------------------------
"""
LMB2 frames as seen by the Outside Network Area (ONA).

The ONA does not trust its gateways: it re-checks every frame they pass on
with its own copy of the mission key before the frame can count in a vote.

Frame types (all 44 bytes, little-endian, same header and same MAC rule):

    0x21 RECORD   a beacon's record (the Writer's memory)          beacon -> all
    0x22 DIGEST   a gateway's "what I hold" list (pull)            gateway -> beacons
    0x23 ROBOT    a robot's position report ("ping")               robot -> gateways
    0x24 MISSION  a mission briefing for the Executor              ONA -> gateways -> Executor

    MAC = HMAC-SHA256(key, "LMB2" || b[0..1] || b[6..35])[0..7]  at bytes 36..43

Bytes 2..5 (relay id, hops|limit, header flags) change at each radio hop and
are not signed.  Everything else is.  The version/type byte and the network id
are inside the MAC, so a frame of one type can never be passed off as another.

RECORD layout: see beacon_net/ARCHITECTURE.md section 2 (this file decodes it
byte for byte like beacon_net/python/beaconnet/proto.py).

ROBOT layout (new, v9):

    off size field
      6  2  robot_id        u16
      8  2  seq             u16, +1 per report
     10  1  role<<4|phase   role 0 Writer, 1 Executor; phase = state index
     11  1  flags           POSE_VALID 1, STUCK 2, LOW_BATT 4, MISSION_ACK 8, EMERGENCY 16
     12  4  x_mm            i32, private map frame (SLAM), millimetres
     16  4  y_mm            i32
     20  2  yaw             u16, 65536 = one turn, map frame
     22  2  pose_sd_cm      u16, the robot's own 1-sigma position error (0xFFFF unknown)
     24  2  odo_dm          u16, distance driven, 0.1 m, wraps
     26  1  battery_pct     255 = unknown
     27  1  done<<4|total   targets treated / targets in the mission (0..15 each)
     28  4  timestamp       unix seconds
     32  2  mission_id      u16, the briefing being executed (0 = none)
     34  2  last_beacon     u16, last beacon passed (0xFFFF = none)

MISSION layout (new, v9):

      6  2  mission_id      u16
      8  2  seq             u16, version of this mission (a re-dispatch bumps it)
     10  2  robot_id        u16, 0xFFFF = any Executor
     12  1  count|flags<<4  count 0..9 targets in this frame;
                            flags RETURN_TO_EXIT 1, ABORT 2, IN_ORDER 4
     13  1  part<<4|parts   this frame's index and the number of frames (0..15)
     14  4  timestamp       unix seconds
     18 18  targets         9 x u16 beacon ids, unused = 0xFFFF
"""
from __future__ import annotations

import hashlib
import hmac
import math
import struct
import time
from dataclasses import dataclass
from typing import Optional

FRAME_LEN = 44
T_RECORD, T_DIGEST, T_ROBOT, T_MISSION = 0x21, 0x22, 0x23, 0x24
TYPE_NAME = {T_RECORD: 'record', T_DIGEST: 'digest', T_ROBOT: 'robot', T_MISSION: 'mission'}
NET_DEFAULT = 0x2A
KEY_DEMO = b'LivingMap-DEMO!!'
DOMAIN = b'LMB2'
NONE16 = 0xFFFF

KINDS = ['WAYPOINT', 'VICTIM', 'GAS', 'RADIATION', 'THERMAL', 'OBSTRUCTION', 'STRUCTURAL', 'EXIT',
         'PHOSPHATE', 'GOLD', 'GEMSTONE', 'SEARCHED']
# kind -> (half-life s, ttl s, gossip weight), as beacon_proto.c
KIND_TABLE = {
    'WAYPOINT': (43200, 86400, 1.0), 'VICTIM': (3600, 7200, 3.0), 'GAS': (600, 3600, 2.5),
    'RADIATION': (21600, 43200, 2.5), 'THERMAL': (1200, 3600, 2.0), 'OBSTRUCTION': (172800, 604800, 1.5),
    'STRUCTURAL': (86400, 259200, 2.0), 'EXIT': (172800, 604800, 1.5),
    'PHOSPHATE': (259200, 604800, 0.8), 'GOLD': (259200, 604800, 0.8), 'GEMSTONE': (259200, 604800, 0.8),
    'SEARCHED': (7200, 43200, 1.2),
}
F_HAS_NEXT, F_HAS_BEARING, F_POS_GNSS, F_RETRACTED, F_STALE = 0x01, 0x02, 0x04, 0x08, 0x10

ROLES = ['WRITER', 'EXECUTOR', 'OTHER']
PHASES = {
    'WRITER': ['WAIT', 'PLAN', 'FOLLOW', 'LOOK', 'NUDGE', 'RETREAT', 'RECOVER', 'HOME', 'DONE'],
    'EXECUTOR': ['WAIT', 'GOTO', 'APPROACH', 'TREAT', 'RETURN', 'DONE', 'RECOVER'],
    'OTHER': ['IDLE'],
}
RF_POSE_VALID, RF_STUCK, RF_LOW_BATT, RF_MISSION_ACK, RF_EMERGENCY = 0x01, 0x02, 0x04, 0x08, 0x10
MF_RETURN, MF_ABORT, MF_IN_ORDER = 0x1, 0x2, 0x4
MISSION_TARGETS_PER_FRAME = 9

_REC = struct.Struct('<BBHBBHHBBiiBBBBHHIHH')     # bytes 0..35 of a RECORD
_ROB = struct.Struct('<BBHBBHHBBiiHHHBBIHH')      # bytes 0..35 of a ROBOT
_MIS = struct.Struct('<BBHBBHHHBBI9H')            # bytes 0..35 of a MISSION
assert _REC.size == _ROB.size == _MIS.size == 36


class DecodeError(Exception):
    """Frame refused. .reason is a short machine word (bad_len, bad_ver, bad_mac, bad_net, bad_field)."""

    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}{": " + detail if detail else ""}')
        self.reason = reason


# ---------------------------------------------------------------------- keys / MAC
def parse_key(k) -> bytes:
    """'demo' -> the demo key; 32 hex characters -> 16 bytes; bytes pass through."""
    if isinstance(k, (bytes, bytearray)):
        if len(k) != 16:
            raise ValueError('mission key must be 16 bytes')
        return bytes(k)
    k = str(k).strip()
    if k.lower() == 'demo':
        return KEY_DEMO
    b = bytes.fromhex(k)
    if len(b) != 16:
        raise ValueError('mission key must be 32 hex characters (128 bits)')
    return b


def mac8(key: bytes, frame: bytes) -> bytes:
    return hmac.new(key, DOMAIN + bytes(frame[0:2]) + bytes(frame[6:36]), hashlib.sha256).digest()[:8]


def seal(key: bytes, body36: bytes) -> bytes:
    """36 bytes (header + signed body) -> 44-byte frame with its MAC."""
    return bytes(body36) + mac8(key, body36)


def verify(key: bytes, frame: bytes) -> bool:
    return len(frame) == FRAME_LEN and hmac.compare_digest(mac8(key, frame), bytes(frame[36:44]))


def sealed_part(frame: bytes) -> bytes:
    """The part of a frame that is identical in every copy of it: type, net, signed body, MAC.
    Two gateways that heard the same transmission (even through different relays) report
    exactly these bytes. This is what the ONA votes on."""
    return bytes(frame[0:2]) + bytes(frame[6:44])


def frame_type(frame: bytes) -> int:
    return frame[0] if frame else 0


# ---------------------------------------------------------------------- helpers
# the C codec's rounding (beacon_proto.c), so that a record encoded here is byte-identical
def _half_up(x):
    return int(x + 0.5) if x >= 0 else 0


def _u8(v, lo=0, hi=255):
    return max(lo, min(hi, _half_up(float(v))))


def _deg_e7(d):
    return int(d * 1e7 + 0.5) if d >= 0 else int(d * 1e7 - 0.5)


def _bearing_to_byte(deg):
    d = math.fmod(float(deg), 360.0)
    if d < 0:
        d += 360.0
    return int(d * 256.0 / 360.0 + 0.5) & 0xFF


def _s_to_10s(s):
    return min(max((int(s) + 5) // 10, 1), 0xFFFF)


@dataclass
class Header:
    ftype: int
    net_id: int
    relay_id: int
    hops: int
    limit: int
    hdr_flags: int


def _header(frame: bytes) -> Header:
    return Header(frame[0], frame[1], frame[2] | (frame[3] << 8), frame[4] >> 4, frame[4] & 0x0F, frame[5])


def _check(frame, key, net_id, expect_type=None):
    if frame is None or len(frame) != FRAME_LEN:
        raise DecodeError('bad_len', f'{0 if frame is None else len(frame)} bytes')
    if frame[0] >> 4 != 2 or frame[0] not in TYPE_NAME:
        raise DecodeError('bad_ver', f'0x{frame[0]:02x}')
    if expect_type is not None and frame[0] != expect_type:
        raise DecodeError('bad_ver', f'0x{frame[0]:02x}')
    if frame[0] == T_DIGEST:
        raise DecodeError('bad_ver', 'digest frames are not decoded by the ONA')
    if key is not None and not verify(key, frame):
        raise DecodeError('bad_mac')
    if net_id is not None and frame[1] != net_id:
        raise DecodeError('bad_net', f'{frame[1]} != {net_id}')


# ---------------------------------------------------------------------- RECORD
def decode_record(frame: bytes, key: Optional[bytes] = KEY_DEMO, net_id: Optional[int] = None) -> dict:
    frame = bytes(frame)
    _check(frame, key, net_id, T_RECORD)
    (vt, net, relay, hl, hf, origin, seq, kind, flags, lat_e7, lon_e7, err_dm, sev, conf, brg,
     next_id, next_dm, ts, ttl10, half10) = _REC.unpack(frame[:36])
    if kind >= len(KINDS) or abs(lat_e7) > 900000000 or abs(lon_e7) > 1800000000:
        raise DecodeError('bad_field', f'kind {kind} lat {lat_e7} lon {lon_e7}')
    has_next = bool(flags & F_HAS_NEXT) and next_id != NONE16
    rec = {
        'beacon_id': origin, 'seq': seq, 'kind': KINDS[kind],
        'gps': {'lat': lat_e7 / 1e7, 'lon': lon_e7 / 1e7, 'err_m': None if err_dm == 255 else err_dm / 10.0,
                'src': 'gnss' if flags & F_POS_GNSS else 'slam'},
        'severity': sev, 'confidence': round(conf / 255.0, 4),
        'next': ({'id': next_id, 'dist_m': next_dm / 10.0,
                  'bearing_deg': (round(brg * 360.0 / 256.0, 1) if flags & F_HAS_BEARING else None)}
                 if has_next else None),
        'ts': ts, 'ttl_s': ttl10 * 10, 'half_life_s': half10 * 10,
        'retracted': bool(flags & F_RETRACTED), 'stale': bool(flags & F_STALE),
    }
    h = Header(vt, net, relay, hl >> 4, hl & 0x0F, hf)
    rec['_hdr'] = {'relay': h.relay_id, 'hops': h.hops, 'limit': h.limit, 'net_id': net,
                   'silent': bool(hf & 0x01), 'alive': bool(hf & 0x02)}
    return rec


def encode_record(rec: dict, key: bytes = KEY_DEMO, net_id: int = NET_DEFAULT, relay: Optional[int] = None,
                  hops: int = 0, limit: int = 15, hdr_flags: int = 0) -> bytes:
    """JSON form (as in ARCHITECTURE.md) -> 44-byte frame. Defaults as the Python/C codec."""
    kind = str(rec.get('kind', 'WAYPOINT')).upper()
    if kind not in KINDS:
        raise ValueError(f'unknown kind {kind}')
    gps = rec.get('gps') or {}
    hl_def, ttl_def, _w = KIND_TABLE[kind]
    flags = 0
    nxt = rec.get('next')
    next_id, next_dm, brg = NONE16, 0, 0
    if nxt and nxt.get('id') is not None:
        flags |= F_HAS_NEXT
        next_id = int(nxt['id'])
        dist = max(0.0, float(nxt.get('dist_m') or 0))
        next_dm = 0xFFFF if dist >= 6553.5 else _half_up(dist * 10.0)
        if nxt.get('bearing_deg') is not None:
            flags |= F_HAS_BEARING
            brg = _bearing_to_byte(float(nxt['bearing_deg']))
    if str(gps.get('src', 'slam')).lower() == 'gnss':
        flags |= F_POS_GNSS
    if rec.get('retracted'):
        flags |= F_RETRACTED
    if rec.get('stale'):
        flags |= F_STALE
    err = gps.get('err_m')
    err_dm = 255 if err is None else (254 if float(err) >= 25.4 else _half_up(max(0.0, float(err)) * 10.0))
    origin = int(rec['beacon_id'])
    body = _REC.pack(T_RECORD, net_id & 0xFF, origin if relay is None else relay, ((hops & 0xF) << 4) | (limit & 0xF),
                     hdr_flags & 0xFF, origin, int(rec.get('seq', 1)) & 0xFFFF, KINDS.index(kind), flags,
                     _deg_e7(float(gps['lat'])), _deg_e7(float(gps['lon'])), err_dm,
                     _u8(rec.get('severity', 0)), _u8(float(rec.get('confidence', 1.0)) * 255), brg, next_id, next_dm,
                     int(rec.get('ts', time.time())) & 0xFFFFFFFF,
                     _s_to_10s(float(rec.get('ttl_s', ttl_def))),
                     _s_to_10s(float(rec.get('half_life_s', hl_def))))
    return seal(key, body)


def eff_conf(rec: dict, now: float) -> float:
    age = now - rec['ts']
    if age > rec['ttl_s']:
        return 0.0
    hl = max(1.0, float(rec['half_life_s']))
    return rec['confidence'] * math.pow(2.0, -max(0.0, age) / hl)


def priority(rec: dict, now: float) -> float:
    """Same value rule as the gossip engine: weight(kind) x (1 + severity/64) x eff_conf."""
    return KIND_TABLE[rec['kind']][2] * (1 + rec['severity'] / 64.0) * eff_conf(rec, now)


# ---------------------------------------------------------------------- ROBOT
def encode_robot(r: dict, key: bytes = KEY_DEMO, net_id: int = NET_DEFAULT) -> bytes:
    role = str(r.get('role', 'EXECUTOR')).upper()
    role_i = ROLES.index(role) if role in ROLES else 2
    phases = PHASES[ROLES[role_i]]
    ph = str(r.get('phase', phases[0])).upper()
    ph_i = phases.index(ph) if ph in phases else 0
    flags = 0
    for name, bit in (('pose_valid', RF_POSE_VALID), ('stuck', RF_STUCK), ('low_batt', RF_LOW_BATT),
                      ('mission_ack', RF_MISSION_ACK), ('emergency', RF_EMERGENCY)):
        if r.get(name, name == 'pose_valid'):
            flags |= bit
    sd = r.get('pose_sd_m')
    body = _ROB.pack(T_ROBOT, net_id & 0xFF, int(r['robot_id']) & 0xFFFF, 0x00, 0,
                     int(r['robot_id']) & 0xFFFF, int(r.get('seq', 1)) & 0xFFFF, (role_i << 4) | (ph_i & 0xF), flags,
                     int(round(float(r['x']) * 1000)), int(round(float(r['y']) * 1000)),
                     int(round((float(r.get('yaw', 0.0)) % (2 * math.pi)) / (2 * math.pi) * 65536)) & 0xFFFF,
                     NONE16 if sd is None else min(65534, max(0, int(round(float(sd) * 100)))),
                     int(round(float(r.get('odo_m', 0.0)) * 10)) & 0xFFFF,
                     255 if r.get('battery_pct') is None else _u8(r['battery_pct'], 0, 100),
                     (min(15, int(r.get('done', 0))) << 4) | min(15, int(r.get('total', 0))),
                     int(r.get('ts', time.time())) & 0xFFFFFFFF,
                     int(r.get('mission_id', 0)) & 0xFFFF,
                     NONE16 if r.get('last_beacon') is None else int(r['last_beacon']) & 0xFFFF)
    return seal(key, body)


def decode_robot(frame: bytes, key: Optional[bytes] = KEY_DEMO, net_id: Optional[int] = None) -> dict:
    frame = bytes(frame)
    _check(frame, key, net_id, T_ROBOT)
    (vt, net, relay, hl, hf, rid, seq, rp, flags, x_mm, y_mm, yaw, sd, odo, batt, prog, ts, mid,
     last) = _ROB.unpack(frame[:36])
    role_i = rp >> 4
    if role_i >= len(ROLES):
        raise DecodeError('bad_field', f'role {role_i}')
    role = ROLES[role_i]
    phases = PHASES[role]
    return {
        'robot_id': rid, 'seq': seq, 'role': role,
        'phase': phases[rp & 0xF] if (rp & 0xF) < len(phases) else f'P{rp & 0xF}',
        'pose_valid': bool(flags & RF_POSE_VALID), 'stuck': bool(flags & RF_STUCK),
        'low_batt': bool(flags & RF_LOW_BATT), 'mission_ack': bool(flags & RF_MISSION_ACK),
        'emergency': bool(flags & RF_EMERGENCY),
        'x': x_mm / 1000.0, 'y': y_mm / 1000.0, 'yaw': yaw / 65536.0 * 2 * math.pi,
        'pose_sd_m': None if sd == NONE16 else sd / 100.0, 'odo_m': odo / 10.0,
        'battery_pct': None if batt == 255 else batt, 'done': prog >> 4, 'total': prog & 0xF,
        'ts': ts, 'mission_id': mid, 'last_beacon': None if last == NONE16 else last,
        '_hdr': {'relay': relay, 'hops': hl >> 4, 'limit': hl & 0xF, 'net_id': net},
    }


# ---------------------------------------------------------------------- MISSION
def encode_mission(mission_id: int, targets, key: bytes = KEY_DEMO, net_id: int = NET_DEFAULT, seq: int = 1,
                   robot_id: int = NONE16, return_to_exit: bool = True, abort: bool = False,
                   in_order: bool = True, ts: Optional[int] = None, relay: int = 0) -> list:
    """A briefing -> one or more signed 44-byte frames (9 targets per frame, at most 15 frames)."""
    targets = [int(t) for t in targets]
    parts = max(1, math.ceil(len(targets) / MISSION_TARGETS_PER_FRAME))
    if parts > 15:
        raise ValueError('a briefing holds at most 135 targets')
    flags = (MF_RETURN if return_to_exit else 0) | (MF_ABORT if abort else 0) | (MF_IN_ORDER if in_order else 0)
    ts = int(time.time() if ts is None else ts)
    out = []
    for p in range(parts):
        chunk = targets[p * MISSION_TARGETS_PER_FRAME:(p + 1) * MISSION_TARGETS_PER_FRAME]
        slots = chunk + [NONE16] * (MISSION_TARGETS_PER_FRAME - len(chunk))
        body = _MIS.pack(T_MISSION, net_id & 0xFF, relay & 0xFFFF, 0x00, 0, mission_id & 0xFFFF, seq & 0xFFFF,
                         robot_id & 0xFFFF, (flags << 4) | len(chunk), (p << 4) | parts, ts & 0xFFFFFFFF, *slots)
        out.append(seal(key, body))
    return out


def decode_mission(frame: bytes, key: Optional[bytes] = KEY_DEMO, net_id: Optional[int] = None) -> dict:
    frame = bytes(frame)
    _check(frame, key, net_id, T_MISSION)
    vals = _MIS.unpack(frame[:36])
    (vt, net, relay, hl, hf, mid, seq, rid, cf, pp, ts) = vals[:11]
    slots = vals[11:]
    count, flags = cf & 0xF, cf >> 4
    if count > MISSION_TARGETS_PER_FRAME or (pp & 0xF) == 0 or (pp >> 4) >= (pp & 0xF):
        raise DecodeError('bad_field', f'count {count} part {pp >> 4}/{pp & 0xF}')
    return {'mission_id': mid, 'seq': seq, 'robot_id': None if rid == NONE16 else rid,
            'targets': list(slots[:count]), 'return_to_exit': bool(flags & MF_RETURN),
            'abort': bool(flags & MF_ABORT), 'in_order': bool(flags & MF_IN_ORDER),
            'part': pp >> 4, 'parts': pp & 0xF, 'ts': ts, '_hdr': {'relay': relay, 'net_id': net}}


class MissionAssembler:
    """Collects the parts of a multi-frame briefing; returns the whole mission once complete."""

    def __init__(self):
        self._parts = {}

    def add(self, m: dict) -> Optional[dict]:
        k = (m['mission_id'], m['seq'])
        d = self._parts.setdefault(k, {})
        d[m['part']] = m
        if len(d) == m['parts']:
            whole = dict(d[0])
            whole['targets'] = [t for i in range(m['parts']) for t in d[i]['targets']]
            whole.pop('part', None)
            del self._parts[k]
            return whole
        return None


# ---------------------------------------------------------------------- any frame
def decode_any(frame: bytes, key: Optional[bytes] = KEY_DEMO, net_id: Optional[int] = None) -> tuple:
    """-> (type name, decoded dict). Raises DecodeError."""
    frame = bytes(frame)
    if len(frame) != FRAME_LEN:
        raise DecodeError('bad_len', f'{len(frame)} bytes')
    t = frame[0]
    if t == T_RECORD:
        return 'record', decode_record(frame, key, net_id)
    if t == T_ROBOT:
        return 'robot', decode_robot(frame, key, net_id)
    if t == T_MISSION:
        return 'mission', decode_mission(frame, key, net_id)
    raise DecodeError('bad_ver', f'0x{t:02x}')


def record_signed_fields(rec: dict) -> tuple:
    """Canonical tuple of a record's signed content, at the precision a gateway prints its JSON lines
    (bp_entry_to_json). A frame the ONA decoded itself and the same record printed as JSON by a gateway
    give the same tuple, so the vote works with old and new gateway firmware alike, and a gateway that
    reports a record twice (raw frame + decoded event) still counts once."""
    g = rec.get('gps') or {}
    n = rec.get('next') or {}
    return (int(rec['beacon_id']), int(rec['seq']), str(rec['kind']).upper(),
            round(float(g.get('lat', 0)), 7), round(float(g.get('lon', 0)), 7),
            None if g.get('err_m') is None else round(float(g['err_m']), 1), str(g.get('src', 'slam')),
            int(rec.get('severity', 0)), round(float(rec.get('confidence', 1.0)), 2),
            n.get('id'), None if n.get('dist_m') is None else round(float(n['dist_m']), 1),
            None if n.get('bearing_deg') is None else round(float(n['bearing_deg']), 1),
            int(rec.get('ts', 0)), int(rec.get('ttl_s', 0)), int(rec.get('half_life_s', 0)),
            bool(rec.get('retracted')), bool(rec.get('stale')))
