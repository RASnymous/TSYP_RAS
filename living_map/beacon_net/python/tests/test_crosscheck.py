#!/usr/bin/env python3
"""
Cross-check the Python mirror against the C implementation (the one the ESP32
firmware runs), byte for byte. Run from anywhere:

    python3 python/tests/test_crosscheck.py        (needs host/bptool: make -C host)

Checks
  1. the reference record from the brief -> identical 44-byte frame
  2. 400 random records: C encode == Python encode; each side decodes the other
  3. authentication: wrong key, flipped record bit, foreign net -> rejected;
     unsigned header (hops / relay) may change without breaking the MAC
  4. 300 random mission-log JSON objects: C parser == Python parser (frames
     identical, and both reject the same malformed inputs)
  5. HMAC-SHA256 (portable C) == Python hmac/hashlib, incl. long keys
  6. digest frames both ways
  7. LoRa time-on-air formula, every SF / BW / CR / payload length
  8. the real beacon firmware (compiled for the PC) transmits the same frame
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "python"))
from beaconnet import proto  # noqa: E402

def _find(*cands):
    for c in cands:
        if os.path.exists(c):
            return c
    return cands[0]


# Linux/WSL: built by `make -C host`.  Windows: the prebuilt windows/bptool.exe
if os.name == "nt":
    BPTOOL = _find(os.path.join(ROOT, "host", "bptool.exe"), os.path.join(ROOT, "windows", "bptool.exe"))
else:
    BPTOOL = os.path.join(ROOT, "host", "bptool")
FW_BEACON = _find(os.path.join(ROOT, "host", "fw_beacon"), os.path.join(ROOT, "host", "fw_beacon.exe"))
REF_JSON = ('{"beacon_id":7,"kind":"VICTIM","gps":{"lat":34.4312,"lon":8.7845,"err_m":2.1},'
            ' "severity":15,"next":{"id":6,"dist_m":8.5},"ttl_s":7200}')
REF_FRAME = "212a07000f00070001000101c0c8851488683c05150fff0006005500a094d068d0026801a283eea13e02c0f2"

rng = random.Random(20260925)
fails = 0


def check(cond: bool, what: str) -> None:
    global fails
    print(f"  [{'PASS' if cond else 'FAIL'}] {what}")
    fails += 0 if cond else 1


def tool(*args: str) -> str:
    return subprocess.run([BPTOOL, *map(str, args)], capture_output=True, text=True).stdout.strip()


def rand_record() -> proto.Record:
    flags = rng.choice([0, proto.RF_HAS_NEXT, proto.RF_HAS_NEXT | proto.RF_HAS_BEARING]) \
        | rng.choice([0, proto.RF_POS_GNSS]) | rng.choice([0, 0, proto.RF_RETRACTED]) \
        | rng.choice([0, 0, proto.RF_STALE])
    return proto.Record(
        origin_id=rng.randrange(0, 0xFFFF), seq=rng.randrange(0, 0x10000), kind=rng.randrange(0, 8),
        flags=flags, lat_e7=rng.randrange(-900000000, 900000001), lon_e7=rng.randrange(-1800000000, 1800000001),
        err_dm=rng.randrange(0, 256), severity=rng.randrange(0, 256), confidence=rng.randrange(0, 256),
        next_bearing=rng.randrange(0, 256),
        next_id=rng.randrange(0, 0xFFFF) if flags & proto.RF_HAS_NEXT else rng.choice([proto.ID_NONE, 3]),
        next_dist_dm=rng.randrange(0, 0x10000), timestamp=rng.randrange(0, 2**32),
        ttl_10s=rng.randrange(1, 0x10000), half_life_10s=rng.randrange(1, 0x10000))


def main() -> int:
    if not os.path.exists(BPTOOL):
        print("bptool not found: run  make -C host  (Linux/WSL), or use windows/bptool.exe")
        return 2
    key = proto.DEMO_KEY

    print("1. reference record (the brief's JSON)")
    rec = proto.record_from_json(REF_JSON, now=1758500000)
    py = proto.encode(proto.Frame(rec=rec, net_id=0x2A, relay_id=7), key).hex()
    c_line = [l for l in tool("example").splitlines() if l.startswith("frame")][0].split(":")[1].strip()
    check(py == c_line == REF_FRAME, f"Python == C == documented frame ({len(bytes.fromhex(py))} bytes)")
    d = proto.record_to_dict(rec)
    check(d["kind"] == "VICTIM" and d["gps"]["err_m"] == 2.1 and d["next"] == {"id": 6, "dist_m": 8.5}
          and d["ttl_s"] == 7200 and d["half_life_s"] == 3600, "decoded fields match the brief")

    print("2. random records, C <-> Python")
    enc_ok = dec_ok = 0
    N = 400
    for _ in range(N):
        r = rand_record()
        k = bytes(rng.randrange(256) for _ in range(16))
        net, relay = rng.randrange(256), rng.randrange(0x10000)
        lim = rng.randrange(0, 16)
        hops = rng.randrange(0, lim + 1)
        hfl = rng.randrange(0, 2)
        f = proto.Frame(rec=r, net_id=net, relay_id=relay, hops=hops, hop_limit=lim, hdr_flags=hfl)
        try:
            pyf = proto.encode(f, k).hex()
        except proto.ProtoError:
            pyf = "error bad_field"
        cf = tool("encode", k.hex(), net, relay, hops, lim, hfl, r.origin_id, r.seq, r.kind, r.flags,
                  r.lat_e7, r.lon_e7, r.err_dm, r.severity, r.confidence, r.next_bearing, r.next_id,
                  r.next_dist_dm, r.timestamp, r.ttl_10s, r.half_life_10s)
        enc_ok += pyf == cf
        if not cf.startswith("error"):
            back = proto.decode(bytes.fromhex(cf), k, net)
            cjs = json.loads(tool("decode", k.hex(), net, pyf))
            dec_ok += back.rec == r and back.hops == hops and cjs["record"]["beacon_id"] == r.origin_id \
                and cjs["record"]["seq"] == r.seq and cjs["hops"] == hops and cjs["relay"] == relay
        else:
            dec_ok += 1
    check(enc_ok == N, f"{enc_ok}/{N} frames byte-identical (incl. identical rejections)")
    check(dec_ok == N, f"{dec_ok}/{N} decoded by the other implementation")

    print("3. authentication")
    fr = bytearray(bytes.fromhex(REF_FRAME))
    def rejects(buf, k=key, net=0x2A):
        try:
            proto.decode(bytes(buf), k, net)
            return None
        except proto.ProtoError as e:
            return str(e)
    check(rejects(fr, k=b"LivingMap-DEMO!?") == "bad_mac", "wrong key -> bad_mac")
    bad = bytearray(fr); bad[proto.HDR_LEN + 15] ^= 0x40
    check(rejects(bad) == "bad_mac", "one flipped bit in the severity -> bad_mac")
    check(rejects(fr, net=0x2B) == "foreign_network", "other team's network id -> rejected before MAC")
    hdr = bytearray(fr); hdr[2] = 9; hdr[4] = 0x3F
    check(rejects(hdr) is None and proto.decode(bytes(hdr), key, 0x2A).relay_id == 9,
          "relay id / hop count re-written by a relay -> still authentic (unsigned header)")
    check(rejects(fr[:43]) == "bad_length", "truncated frame -> bad_length")
    check(tool("decode", key.hex(), 0x2A, bad.hex()) == '{"error":"bad_mac"}', "C agrees: bad_mac")

    print("4. mission-log JSON parser, C <-> Python")
    same = 0
    M = 300
    kinds = proto.KINDS + ["fire", "Heat", "rubble", "trail", "unicorn"]
    for i in range(M):
        dd: dict = {"beacon_id": rng.randrange(0, 70000)}
        if rng.random() < 0.9:
            dd["gps"] = {"lat": round(rng.uniform(-95, 95), rng.randrange(1, 8)),
                         "lon": round(rng.uniform(-185, 185), rng.randrange(1, 8))}
            if rng.random() < 0.7:
                dd["gps"]["err_m"] = rng.choice([None, round(rng.uniform(0, 30), 2), 0, 25.4, 25.35])
            if rng.random() < 0.3:
                dd["gps"]["src"] = rng.choice(["gnss", "slam", "gps"])
        if rng.random() < 0.8:
            dd["kind"] = rng.choice(kinds) if rng.random() < 0.85 else rng.randrange(0, 9)
        if rng.random() < 0.6:
            dd["severity"] = rng.choice([rng.randrange(0, 256), 300, 12.5])
        if rng.random() < 0.5:
            dd["confidence"] = rng.choice([round(rng.random(), 3), 1, 0, 1.2])
        if rng.random() < 0.6:
            n = {"id": rng.randrange(0, 66000)}
            if rng.random() < 0.8:
                n["dist_m"] = round(rng.uniform(0, 7000), rng.randrange(0, 3))
            if rng.random() < 0.5:
                n["bearing_deg"] = round(rng.uniform(-720, 720), 1)
            dd["next"] = n
        elif rng.random() < 0.3:
            dd["next"] = None
        for key_, lo, hi in (("ts", 1.7e9, 1.8e9), ("ttl_s", 1, 700000), ("half_life_s", 1, 700000)):
            if rng.random() < 0.4:
                dd[key_] = rng.choice([int(rng.uniform(lo, hi)), round(rng.uniform(lo, hi), 1)])
        if rng.random() < 0.2:
            dd["retracted"] = rng.choice([True, False])
        if rng.random() < 0.2:
            dd["ev"], dd["rx_node"], dd["hops"] = "new", 900, 3      # gateway extras are ignored
        js = json.dumps(dd)
        try:
            r = proto.record_from_json(js, now=1758500000)
            py = proto.encode(proto.Frame(rec=r, net_id=42, relay_id=r.origin_id), key).hex()
        except proto.ProtoError:
            py = "error"
        out = tool("fromjson", key.hex(), 42, js, 1758500000).splitlines()
        cc = "error" if not out or out[0].startswith("{") else out[0]
        if cc == py:
            same += 1
        elif fails < 3:
            print("     mismatch:", js, "\n       C :", cc, "\n       Py:", py)
    check(same == M, f"{same}/{M} JSON objects give identical frames / identical rejections")

    print("5. HMAC-SHA256")
    ok = 0
    for i in range(60):
        k = bytes(rng.randrange(256) for _ in range(rng.choice([1, 16, 32, 64, 65, 100])))
        m = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 300)))
        ok += tool("hmac", k.hex(), m.hex()) == hmac.new(k, m, hashlib.sha256).hexdigest()
    check(ok == 60, f"{ok}/60 HMACs identical (keys 1..100 bytes, messages 0..300 bytes)")

    print("6. digest frames")
    ok = 0
    for i in range(40):
        ents = sorted({rng.randrange(0, 0xFFFF): rng.randrange(0, 0x10000) for _ in range(rng.randrange(0, 61))}.items())[:60]
        page, fin = rng.randrange(0, 128), rng.random() < 0.5
        pb = proto.encode_digest(42, 900, ents, key, page=page, final=fin)
        cb = tool("digest", key.hex(), 42, 900, page | (0x80 if fin else 0), *[f"{o}:{s}" for o, s in ents])
        dec = proto.decode_digest(bytes.fromhex(cb), key, 42)
        cj = json.loads(tool("decode", key.hex(), 42, pb.hex()))
        ok += pb.hex() == cb and dec == (900, page, fin, ents) and [tuple(e) for e in cj["entries"]] == ents
    check(ok == 40, f"{ok}/40 digests identical and cross-decoded")

    print("7. LoRa time-on-air")
    ok = tot = 0
    for sf in range(7, 13):
        for bw in (125000, 250000, 500000):
            for cr in (5, 6, 7, 8):
                for pl in (1, 14, 44, 100, 134, 206, 255):
                    tot += 1
                    ok += int(tool("airtime", pl, sf, bw, cr)) == proto.lora_airtime_ms(pl, sf, bw, cr)
    check(ok == tot, f"{ok}/{tot} airtime values identical")
    check(proto.lora_airtime_ms(44, 9) == 288 and proto.lora_airtime_ms(44, 7) == 93
          and proto.lora_airtime_ms(44, 12) == 2139, "44 B: SF7 93 ms, SF9 288 ms, SF12 2139 ms")

    print("8. firmware (the real sketch, built for the PC)")
    if os.path.exists(FW_BEACON):
        nvs = os.path.join(ROOT, "host", "nvs_test.bin")
        if os.path.exists(nvs):
            os.remove(nvs)
        cmds = f"KEY demo\nTIME 1758500000\nPROV {REF_JSON}\n"
        p = subprocess.run([FW_BEACON], input=cmds, capture_output=True, text=True, timeout=20,
                           env={**os.environ, "NVS_FILE": nvs})
        tx = [l.split()[-1] for l in p.stderr.splitlines() if l.startswith("TX")]
        check(bool(tx) and tx[0] == REF_FRAME, "beacon firmware's first transmission == reference frame")
        check("OK PROV id=7 seq=1" in p.stdout, "firmware acknowledged provisioning (id 7, seq 1)")
        p2 = subprocess.run([FW_BEACON], input=f"TIME 1758500100\nPROV {REF_JSON}\n", capture_output=True, text=True,
                            timeout=20, env={**os.environ, "NVS_FILE": nvs})
        check("OK PROV id=7 seq=2" in p2.stdout, "after a reboot, re-provisioning bumps seq (no version reuse)")
        os.remove(nvs)
    else:
        print("  [SKIP] host/fw_beacon not built (make -C host firmware-host)")

    print("\nRESULT:", "ALL PASSED" if not fails else f"{fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
