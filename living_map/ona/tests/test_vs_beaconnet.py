#!/usr/bin/env python3
"""
The ONA's frame codec against the beacon network's own (beacon_net/python/beaconnet/proto.py,
itself checked byte for byte against the C library the ESP32 firmware runs).

  1. 600 random records: both encoders give the same 44 bytes; each side decodes the other's frame
  2. the canonical "signed fields" the ONA votes on are the same whether a gateway passed
     the raw frame or its JSON line (bp_entry_to_json precision)
  3. a ROBOT or MISSION frame is never taken for a record by the beacon codec (and the
     beacons' gossip ignores them: bp_gossip.c counts them as rx_foreign)

    python3 tests/test_vs_beaconnet.py
"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from ona import lmb2  # noqa: E402

BN = None
for cand in (os.path.join(os.path.dirname(ROOT), 'beacon_net', 'python'),
             os.path.expanduser('~/living_map/beacon_net/python')):
    if os.path.exists(os.path.join(cand, 'beaconnet', 'proto.py')):
        BN = cand
        break

RESULTS = []


def check(ok, text):
    RESULTS.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {text}')


def rand_record(rng):
    kind = rng.choice(lmb2.KINDS)
    d = {'beacon_id': rng.randrange(0, 65535), 'seq': rng.randrange(0, 65536), 'kind': kind,
         'gps': {'lat': rng.uniform(-89.9, 89.9), 'lon': rng.uniform(-179.9, 179.9)},
         'ts': rng.randrange(0, 2 ** 32)}
    if rng.random() < 0.8:
        d['gps']['err_m'] = rng.choice([round(rng.uniform(0, 30), 2), 0.25, 0.35, 12.45, 25.4, 40.0])
    if rng.random() < 0.3:
        d['gps']['src'] = 'gnss'
    if rng.random() < 0.8:
        d['severity'] = rng.randrange(0, 256)
    if rng.random() < 0.8:
        d['confidence'] = rng.choice([round(rng.random(), 3), 0.5, 1.0, 0.0, 0.002])
    if rng.random() < 0.7:
        nx = {'id': rng.randrange(0, 65535)}
        if rng.random() < 0.9:
            nx['dist_m'] = rng.choice([round(rng.uniform(0, 80), 2), 0.05, 0.25, 6553.5, 7000.0])
        if rng.random() < 0.8:
            nx['bearing_deg'] = rng.choice([round(rng.uniform(-720, 720), 3), 359.5, 0.703125, -0.1])
        d['next'] = nx
    if rng.random() < 0.6:
        d['ttl_s'] = rng.choice([rng.randrange(1, 600000), 5, 15, 25, 99995])
    if rng.random() < 0.6:
        d['half_life_s'] = rng.choice([rng.randrange(1, 600000), 5, 15, 655350])
    if rng.random() < 0.1:
        d['retracted'] = True
    if rng.random() < 0.1:
        d['stale'] = True
    return d


def main():
    if BN is None:
        print('beacon_net/python not found next to the ONA: nothing to cross-check')
        return 0
    sys.path.insert(0, BN)
    from beaconnet import proto
    key = lmb2.KEY_DEMO
    rng = random.Random(9)
    same = dec_ok = 0
    first_diff = None
    N = 600
    for i in range(N):
        d = rand_record(rng)
        relay, hops = rng.randrange(0, 65535), rng.randrange(0, 16)
        a = proto.encode(proto.Frame(rec=proto.record_from_json(d), net_id=42, relay_id=relay, hops=hops,
                                     hop_limit=15), key)
        b = lmb2.encode_record(d, key, 42, relay=relay, hops=hops, limit=15)
        if a == b:
            same += 1
        elif first_diff is None:
            first_diff = (d, a.hex(), b.hex())
        try:
            f = proto.decode(b, key, 42)
            r = lmb2.decode_record(a, key, 42)
            js = proto.record_to_dict(f.rec)
            ok = (r['beacon_id'] == js['beacon_id'] and r['seq'] == js['seq'] and r['kind'] == js['kind']
                  and abs(r['gps']['lat'] - js['gps']['lat']) < 1e-7 and r['severity'] == js['severity']
                  and r['ts'] == js['ts'] and r['ttl_s'] == js['ttl_s'] and r['half_life_s'] == js['half_life_s'])
            # the vote: frame -> canonical fields == the gateway's JSON line -> canonical fields
            line = dict(js, beacon_id=js['beacon_id'])
            ok = ok and lmb2.record_signed_fields(r) == lmb2.record_signed_fields(
                lmb2.decode_record(lmb2.encode_record(line, key, 42), key, 42))
            dec_ok += ok
        except Exception as e:  # noqa: BLE001
            if first_diff is None:
                first_diff = (d, repr(e), '')
    check(same == N, f'{same}/{N} random records: the ONA and the beacon codec give the same 44 bytes'
          + (f'  first difference: {first_diff}' if first_diff and same != N else ''))
    check(dec_ok == N, f'{dec_ok}/{N}: each side decodes the other\'s frame; the vote key is the same '
                       'for a raw frame and its JSON line')
    rob = lmb2.encode_robot({'robot_id': 1, 'seq': 3, 'role': 'EXECUTOR', 'x': 1.0, 'y': 2.0})
    mis = lmb2.encode_mission(77, [4, 8, 12])[0]
    rejected = 0
    for fr in (rob, mis):
        try:
            proto.decode(fr, key, 42)
        except proto.ProtoError:
            rejected += 1
    check(rejected == 2, 'ROBOT (0x23) and MISSION (0x24) frames are never taken for a beacon record')
    print(f'\n{sum(RESULTS)}/{len(RESULTS)} checks passed')
    return 0 if all(RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
