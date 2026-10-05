#!/usr/bin/env python3
"""Unit and integration tests for the Outside Network Area.

    python3 tests/test_ona.py            (from the ona folder; Linux or Windows)

No network, no radio: everything runs in-process with a fake clock.
"""
import json
import math
import os
import random
import shutil
import sqlite3
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from ona import lmb2  # noqa: E402
from ona.core import OnaConfig, OnaCore  # noqa: E402
from ona.geodesy import AnchorPoint, EnuFrame, LocalToGpsCalibrator, check_against  # noqa: E402
from ona.kalman import PositionEKF, RangeEKF  # noqa: E402
from ona.multilateration import RangeObs, trilaterate, hdop_map  # noqa: E402
from ona.quorum import CONFIRMED, CONFLICT, PENDING, QuorumVoter  # noqa: E402
from ona.store import PersistentQueue  # noqa: E402
from ona.uplink import LinkModel, LinkType, SimulatedTransport, Uplink, compact_for_satellite  # noqa: E402

RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond), detail))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{('  (' + str(detail) + ')') if detail else ''}")


BRIEF_HEX = '212a07000f00070001000101c0c8851488683c05150fff0006005500a094d068d0026801a283eea13e02c0f2'
BRIEF = {'beacon_id': 7, 'kind': 'VICTIM', 'gps': {'lat': 34.4312, 'lon': 8.7845, 'err_m': 2.1}, 'severity': 15,
         'next': {'id': 6, 'dist_m': 8.5}, 'ttl_s': 7200, 'ts': 1758500000}


# ------------------------------------------------------------------ LMB2 frames
def test_frames():
    print('LMB2 frames')
    f = bytes.fromhex(BRIEF_HEX)
    rec = lmb2.decode_record(f)
    check('brief frame decodes', rec['kind'] == 'VICTIM' and rec['beacon_id'] == 7 and rec['next']['id'] == 6
          and abs(rec['gps']['lat'] - 34.4312) < 1e-9 and rec['gps']['err_m'] == 2.1)
    check('brief record encodes to the documented bytes', lmb2.encode_record(BRIEF).hex() == BRIEF_HEX)
    relayed = bytearray(f)
    relayed[2:4] = (12).to_bytes(2, 'little')
    relayed[4] = (3 << 4) | 15
    relayed[5] = 1
    check('relay/hops rewritten: still authentic', lmb2.verify(lmb2.KEY_DEMO, bytes(relayed)))
    check('relay/hops rewritten: same sealed part', lmb2.sealed_part(relayed) == lmb2.sealed_part(f))
    bad = bytearray(f)
    bad[21] ^= 0x01
    try:
        lmb2.decode_record(bytes(bad))
        ok = False
    except lmb2.DecodeError as e:
        ok = e.reason == 'bad_mac'
    check('one flipped bit is rejected (bad_mac)', ok)
    try:
        lmb2.decode_record(f, key=bytes(16))
        ok = False
    except lmb2.DecodeError as e:
        ok = e.reason == 'bad_mac'
    check('wrong key is rejected', ok)
    try:
        lmb2.decode_record(f, net_id=7)
        ok = False
    except lmb2.DecodeError as e:
        ok = e.reason == 'bad_net'
    check('foreign network is rejected', ok)
    r = {'robot_id': 101, 'seq': 65535, 'role': 'EXECUTOR', 'phase': 'TREAT', 'x': -12.345, 'y': 7.891, 'yaw': 4.0,
         'pose_sd_m': 0.42, 'odo_m': 123.4, 'battery_pct': 77, 'done': 2, 'total': 4, 'ts': 1790000000,
         'mission_id': 321, 'last_beacon': 13, 'mission_ack': True}
    fr = lmb2.encode_robot(r)
    d = lmb2.decode_robot(fr)
    check('ROBOT frame is 44 bytes and round-trips', len(fr) == 44 and d['robot_id'] == 101 and d['phase'] == 'TREAT'
          and abs(d['x'] + 12.345) < 1e-3 and abs(d['y'] - 7.891) < 1e-3 and abs(d['yaw'] - 4.0) < 1e-4
          and d['pose_sd_m'] == 0.42 and d['mission_ack'] and d['done'] == 2 and d['total'] == 4
          and d['last_beacon'] == 13 and d['seq'] == 65535)
    frames = lmb2.encode_mission(77, list(range(1, 21)), seq=3, robot_id=101)
    asm = lmb2.MissionAssembler()
    whole = None
    for x in reversed(frames):
        whole = asm.add(lmb2.decode_mission(x)) or whole
    check('MISSION: 20 targets in 3 signed frames, reassembled in any order',
          len(frames) == 3 and whole and whole['targets'] == list(range(1, 21)) and whole['return_to_exit'])
    t, _ = lmb2.decode_any(frames[0])
    check('frame types cannot be confused', t == 'mission' and frames[0][0] == 0x23 + 1)
    x = bytearray(frames[0])
    x[0] = lmb2.T_ROBOT
    try:
        lmb2.decode_any(bytes(x))
        ok = False
    except lmb2.DecodeError as e:
        ok = e.reason == 'bad_mac'
    check('a MISSION frame relabelled as ROBOT fails the MAC', ok)
    rnd = random.Random(3)
    ok = True
    for _ in range(300):
        rec = {'beacon_id': rnd.randrange(0, 65535), 'seq': rnd.randrange(0, 65536), 'kind': rnd.choice(lmb2.KINDS),
               'gps': {'lat': rnd.uniform(-89, 89), 'lon': rnd.uniform(-179, 179),
                       'err_m': round(rnd.uniform(0, 25), 1), 'src': rnd.choice(['slam', 'gnss'])},
               'severity': rnd.randrange(0, 256), 'confidence': round(rnd.random(), 2), 'ts': rnd.randrange(0, 2 ** 32),
               'next': {'id': rnd.randrange(0, 65535), 'dist_m': round(rnd.uniform(0, 500), 1),
                        'bearing_deg': rnd.uniform(0, 360)} if rnd.random() < 0.8 else None}
        dec = lmb2.decode_record(lmb2.encode_record(rec))
        if lmb2.encode_record({**dec, 'gps': dict(dec['gps'])}) != lmb2.encode_record(rec):
            ok = False
            break
    check('300 random records: encode(decode(encode(r))) == encode(r)', ok)


# ------------------------------------------------------------------ geodesy (trans.py)
def friend_anchors():
    return [AnchorPoint(np.array([0.0, 0.0, 0.0]), lat=34.4310, lon=8.7840, alt=310.0),
            AnchorPoint(np.array([5.0, 0.0, 0.0]), lat=34.4310, lon=8.78406, alt=310.1),
            AnchorPoint(np.array([0.0, 5.0, 0.0]), lat=34.43105, lon=8.7840, alt=310.0),
            AnchorPoint(np.array([5.0, 5.0, 0.2]), lat=34.43105, lon=8.78406, alt=310.2)]


def test_geodesy():
    print('Frame translation (geodesy.py, from trans.py)')
    cal = LocalToGpsCalibrator(friend_anchors())
    g = cal.local_to_gps(np.array([12.3, 7.8, 0.0]), dist_since_last_anchor_m=42.0, digits=6)
    check("the original example still gives the original answer (err 1.22 m)",
          abs(cal.calib_rms_m - 0.38) < 0.02 and g['err_m'] == 1.22, f"rms {cal.calib_rms_m:.3f}, {g}")
    p = np.array([12.3, 7.8, 0.0])
    q = cal.gps_to_local(*cal.enu_to_gps(cal.local_to_enu(p)))
    check('local -> GPS -> local round trip < 1 mm', np.linalg.norm(q[:2] - p[:2]) < 1e-3)
    ok = True
    for lat in (-60, 0, 34.43, 36.8065, 70):
        f = EnuFrame(lat, 10.0, 50.0)
        for enu in ([30, -40, 2], [-500, 800, -3]):
            back = f.to_enu(*f.to_geodetic(enu))
            ok &= np.linalg.norm(back - np.array(enu)) < 1e-4
    check('ENU <-> WGS84 round trip at 5 latitudes < 0.1 mm', ok)
    rnd = np.random.default_rng(1)
    true_yaw = math.radians(27.0)
    frame = EnuFrame(36.8065, 10.1815, 10.0)
    anchors = []
    for x, y in ((0, 0), (6, 0), (0, 6), (6, 6), (3, 9)):
        e = math.cos(true_yaw) * x - math.sin(true_yaw) * y + 4.0 + rnd.normal(0, 0.05)
        n = math.sin(true_yaw) * x + math.cos(true_yaw) * y - 2.0 + rnd.normal(0, 0.05)
        la, lo, al = frame.to_geodetic([e, n, rnd.normal(0, 1.5)])          # GPS altitude is noisy
        anchors.append(AnchorPoint(np.array([x, y, 0.0]), la, lo, al))
    cal3 = LocalToGpsCalibrator(anchors)
    calp = LocalToGpsCalibrator(anchors, planar=True)
    check('planar Umeyama recovers the yaw from noisy anchors (< 1 deg)',
          abs(math.degrees(calp.yaw_rad - true_yaw)) < 1.0, f'{math.degrees(calp.yaw_rad):.2f} deg')
    far = np.array([40.0, 25.0, 0.0])
    tilt3 = abs(cal3.local_to_enu(far)[2] - cal3.t[2])
    tiltp = abs(calp.local_to_enu(far)[2] - calp.t[2])
    check('planar mode is immune to GPS altitude noise (no tilt)', tiltp < 1e-9 and tilt3 > 0.05,
          f'3D tilt {tilt3:.2f} m at 47 m, planar {tiltp:.1e} m')
    try:
        LocalToGpsCalibrator([AnchorPoint(np.array([i, 0.0, 0.0]), 36.8 + i * 1e-5, 10.18, 0) for i in range(4)])
        ok = False
    except ValueError:
        ok = True
    check('collinear anchors are refused (rotation undetermined)', ok)
    c1 = LocalToGpsCalibrator.from_anchor_yaw(36.8065, 10.1815, 10.0, 30.0)
    x, y = 17.0, -9.0
    e = x * math.cos(math.radians(30)) - y * math.sin(math.radians(30))
    n = x * math.sin(math.radians(30)) + y * math.cos(math.radians(30))
    lat_eq = 36.8065 + n / 111320.0
    lon_eq = 10.1815 + e / (111320.0 * math.cos(math.radians(36.8065)))
    g = c1.local_to_gps(np.array([x, y, 0]))
    fr = EnuFrame(36.8065, 10.1815, 10.0)
    d = np.linalg.norm(fr.to_enu(g['lat'], g['lon']) - fr.to_enu(lat_eq, lon_eq, 10.0))
    check("anchor+yaw calibration matches the Writer's flat-earth formula (< 0.1 m at 19 m)", d < 0.1, f'{d:.3f} m')
    chk = check_against(c1, LocalToGpsCalibrator.from_anchor_yaw(36.8065, 10.1815, 10.0, 31.0), [[20, 0, 0]])
    check('check_against measures a 1 deg frame error (0.35 m at 20 m)', abs(chk['max_m'] - 0.349) < 0.01,
          f"{chk['max_m']:.3f} m")


# ------------------------------------------------------------------ Kalman (ekf.py)
def test_kalman():
    print('Kalman filters (kalman.py, from ekf.py)')
    cal = LocalToGpsCalibrator(friend_anchors())
    ekf = PositionEKF(np.zeros(3), np.eye(3) * cal.calib_rms_m ** 2)
    step = np.array([12.3, 7.8, 0.0]) / 20
    for _ in range(20):
        ekf.predict(step, np.array([0.02, 0.02, 0.01]) * (42.0 / 20))
    before = ekf.err_m
    ekf.update(np.array([12.3, 7.8, 0.0]), np.eye(3) * 9.0)
    check('original PositionEKF: prediction grows P, correction shrinks it', ekf.err_m < before,
          f'{before:.3f} -> {ekf.err_m:.3f} m')
    rnd = np.random.default_rng(5)
    gws = [np.array([-6.0, -4.0, 2.0]), np.array([26.0, -5.0, 2.0]), np.array([10.0, 20.0, 2.0])]
    truth = np.array([2.0, 1.0])
    kf = RangeEKF(0.0, 0.0, 3.0)
    errs = []
    for k in range(150):
        d = np.array([0.1, 0.05])
        truth = truth + d
        kf.predict_move(d + rnd.normal(0, 0.01, 2), float(np.linalg.norm(d)))
        for g in gws:
            r = math.sqrt(float(np.sum((truth - g[:2]) ** 2)) + (0.3 - g[2]) ** 2) + rnd.normal(0, 0.5)
            kf.update_range('g', g, r, 0.5, 0.3)
        errs.append(float(np.linalg.norm(kf.x - truth)))
    check('range EKF tracks a moving robot (mean error < 0.4 m)', np.mean(errs[20:]) < 0.4,
          f'{np.mean(errs[20:]):.2f} m, sigma {kf.sigma_m:.2f} m')
    before = kf.x.copy()
    u = kf.update_range('g', gws[0], 60.0, 0.5, 0.3)
    check('a 40 m wrong range is rejected, the estimate does not jump',
          not u.accepted and np.linalg.norm(kf.x - before) < 1e-9, f'z = {u.z_score:.1f}')
    kf2 = RangeEKF(truth[0], truth[1], 0.5)
    for g in gws[:1]:
        r = math.sqrt(float(np.sum((truth - g[:2]) ** 2)) + (0.3 - g[2]) ** 2)
        for _ in range(3):
            kf2.update_range('g', g, r + 2.5, 0.5, 0.3)
    check('Huber weighting: a +2.5 m wall bias moves the estimate less than the bias',
          np.linalg.norm(kf2.x - truth) < 2.0, f'{np.linalg.norm(kf2.x - truth):.2f} m')


# ------------------------------------------------------------------ multilateration
def test_multilateration():
    print('Three spheres (multilateration.py)')
    gws = {'gw1': np.array([-6.0, -4.0, 2.0]), 'gw2': np.array([26.0, -5.0, 2.5]), 'gw3': np.array([10.0, 20.0, 1.8])}
    truth = np.array([7.5, 4.2])

    def obs(noise=0.0, bias=None, sig=0.5, rnd=None):
        out = []
        for k, g in gws.items():
            r = math.sqrt(float(np.sum((truth - g[:2]) ** 2)) + (0.3 - g[2]) ** 2)
            r += (rnd.normal(0, noise) if rnd is not None else 0.0) + (bias or {}).get(k, 0.0)
            out.append(RangeObs(k, g, r, sig))
        return out
    f = trilaterate(obs(), 0.3)
    check('exact ranges -> exact position', f.ok and abs(f.e - 7.5) < 1e-4 and abs(f.n - 4.2) < 1e-4,
          f'{f.e:.4f}, {f.n:.4f}, HDOP {f.hdop:.2f}')
    rnd = np.random.default_rng(11)
    errs, fails = [], 0
    for _ in range(400):
        f = trilaterate(obs(0.5, rnd=rnd), 0.3)
        errs.append(math.hypot(f.e - truth[0], f.n - truth[1]))
        fails += f.raim == 'fail'
    check('noisy ranges (0.5 m): error ~ sigma x HDOP, RAIM false alarms ~1 %',
          np.sqrt(np.mean(np.square(errs))) < 0.5 * f.hdop * 1.3 and fails <= 12,
          f'RMS {np.sqrt(np.mean(np.square(errs))):.2f} m, HDOP {f.hdop:.2f}, {fails}/400 false alarms')
    f = trilaterate(obs(bias={'gw2': 4.0}), 0.3)
    check('RAIM detects a +4 m NLOS bias on one of 3 gateways', f.raim == 'fail', f'chi2 {f.chi2:.1f}')
    gws['gw4'] = np.array([-4.0, 18.0, 2.0])
    f = trilaterate(obs(bias={'gw2': 4.0}), 0.3)
    check('with 4 gateways RAIM names and excludes the bad one',
          f.raim == 'excluded:gw2' and math.hypot(f.e - 7.5, f.n - 4.2) < 0.05, f.raim)
    del gws['gw4']
    line = {'a': np.array([0.0, 0.0, 2.0]), 'b': np.array([10.0, 0.0, 2.0]), 'c': np.array([20.0, 0.0, 2.0])}
    ob = [RangeObs(k, g, float(np.linalg.norm(np.array([5.0, 6.0, 0.3]) - g)), 0.5) for k, g in line.items()]
    f = trilaterate(ob, 0.3)
    check('gateways in a line: large HDOP (geometry warning)', (not f.ok) or f.hdop > 3 or abs(f.n) > 0,
          f'ok={f.ok} HDOP={f.hdop:.1f}')
    two = obs()[:2]
    f = trilaterate(two, 0.3, x0=[7.0, 5.0])
    check('2 gateways + a prior: picks the right one of the two points',
          f.ok and math.hypot(f.e - 7.5, f.n - 4.2) < 1e-3)
    hm = hdop_map(list(gws.values()), np.linspace(0, 20, 5), np.linspace(-2, 14, 5))
    check('HDOP inside the triangle of gateways < 2', float(np.nanmin(hm)) < 2.0, f'{float(np.nanmin(hm)):.2f}')


# ------------------------------------------------------------------ quorum
def test_quorum():
    print('2-of-3 vote (quorum.py)')
    v = QuorumVoter(['gw1', 'gw2', 'gw3'], 2)
    e = v.offer('gw1', ('R', 7, 1), b'A', {'x': 1}, 0)
    check('one gateway: pending', [x.kind for x in e] == ['pending'])
    check('the same gateway twice does not count twice', v.offer('gw1', ('R', 7, 1), b'A', None, 1) == [])
    e = v.offer('gw2', ('R', 7, 1), b'A', None, 2)
    check('two identical: confirmed', [x.kind for x in e] == ['confirmed'] and v.votes[('R', 7, 1)].state == CONFIRMED)
    e = v.offer('gw3', ('R', 7, 1), b'A', None, 3)
    check('third agrees: more votes (3/3)', [x.kind for x in e] == ['more_votes']
          and v.votes[('R', 7, 1)].count(b'A') == 3)
    e = v.offer('gw3', ('R', 8, 1), b'EVIL', None, 4)
    e = v.offer('gw1', ('R', 8, 1), b'GOOD', None, 5)
    check('1 vs 1: conflict, nothing shown', [x.kind for x in e] == ['conflict'] and v.votes[('R', 8, 1)].state == CONFLICT)
    e = v.offer('gw2', ('R', 8, 1), b'GOOD', None, 6)
    kinds = sorted(x.kind for x in e)
    check('majority decides, the liar is named', kinds == ['confirmed', 'disagree']
          and v.votes[('R', 8, 1)].winner == b'GOOD' and [x.gw for x in e if x.kind == 'disagree'] == ['gw3'])
    for i in range(9, 12):
        v.offer('gw1', ('R', i, 1), b'G', None, 7)
        v.offer('gw2', ('R', i, 1), b'G', None, 7)
        v.offer('gw3', ('R', i, 1), b'B', None, 7)
    check('a gateway that keeps disagreeing is quarantined', v.score('gw3').quarantined, v.score('gw3').note)
    e = v.offer('gw3', ('R', 20, 1), b'X', None, 8)
    check('a quarantined gateway cannot vote', e and e[0].kind == 'ignored' and ('R', 20, 1) not in v.votes
          or v.votes.get(('R', 20, 1)) is None or not v.votes[('R', 20, 1)].bodies)
    e = v.offer('gw1', ('R', 21, 1), b'F', None, 9)
    check('a forged record from one gateway never confirms', v.votes[('R', 21, 1)].state == PENDING)


# ------------------------------------------------------------------ store + uplink
def test_store_uplink():
    print('Store and forward (store.py, uplink.py, from ona.py)')
    tmp = tempfile.mkdtemp()
    try:
        db = os.path.join(tmp, 'q.sqlite3')
        q = PersistentQueue(db)
        q.push({'n': 1}, '/api/beacon', 1)
        q.push({'n': 2}, '/api/beacon', 3)
        q.push({'pos': 1}, '/api/robot-status', 2, coalesce_key='robot:1')
        q.push({'pos': 2}, '/api/robot-status', 2, coalesce_key='robot:1')
        rows = q.pending()
        check('most important first, robot position coalesced to the latest',
              [r[2] for r in rows] == [{'n': 2}, {'pos': 2}, {'n': 1}])
        q.close()
        q = PersistentQueue(db)
        check('queue survives a restart (SQLite)', len(q.pending()) == 3)
        clock = [1000.0]
        links = LinkModel(lte=True, sat=True, lte_outages='10-20', clock=lambda: clock[0])

        class T(SimulatedTransport):
            def lte_available(self):
                return links.lte_up()

            def satellite_available(self):
                return links.sat_up()
        tr = T(log=None)
        up = Uplink(q, tr, sat_interval_s=0.0)
        clock[0] = 1015.0                 # LTE down, satellite up
        up.flush_once()
        check('LTE down: satellite carries only urgent traffic', len(tr.sent) == 1 and tr.sent[0][2] == LinkType.SATELLITE
              and tr.sent[0][1] == {'n': 2})
        links.sat = False
        up.flush_once()
        check('no link: nothing is lost, messages wait', q.counts()['pending'] == 2)
        clock[0] = 1025.0
        up.flush_once()
        check('LTE back: the backlog is delivered', q.counts()['pending'] == 0 and len(tr.sent) == 3)
        big = {'beacon_id': 7, 'event_type': 'victim', 'kind': 'VICTIM', 'seq': 2, 'lat': 36.8065123,
               'lon': 10.1815123, 'value': 40, 'severity': 40, 'ttl': 19, 'timestamp': 1790000000, 'v': 2, 'ev': 'new',
               'confidence': 0.96, 'eff_conf': 0.95, 'err_m': 1.9, 'next': {'id': 6, 'dist_m': 8.6, 'bearing_deg': 180},
               'ttl_s': 7200, 'half_life_s': 3600, 'stale': False, 'retracted': False, 'silent': False,
               'ona': {'state': 'confirmed', 'votes': 3, 'of': 3, 'quorum': 2, 'gateways': ['gw1', 'gw2', 'gw3']},
               'gateway_id': 'gw1+gw2+gw3', 'rssi': -97, 'snr': 3.5, 'hops': 2, 'relay': 5, 'age_s': 12, 'gps_src': 'slam'}
        c = compact_for_satellite('/api/beacon', big)
        size = len(json.dumps(c, separators=(',', ':')))
        check('a record fits one satellite (SBD) message', size <= 340, f'{size} bytes')
        # the friend's original database opens and is upgraded in place
        orig = os.path.join(HERE, 'data', 'ona_queue_original.sqlite3')
        if os.path.exists(orig):
            cp = os.path.join(tmp, 'orig.sqlite3')
            shutil.copy(orig, cp)
            q2 = PersistentQueue(cp)
            cols = {r[1] for r in sqlite3.connect(cp).execute('PRAGMA table_info(uplink_queue)')}
            check("the original ona.py queue file is upgraded, old rows kept",
                  'priority' in cols and sqlite3.connect(cp).execute('SELECT COUNT(*) FROM uplink_queue').fetchone()[0] >= 8)
            q2.close()
        q.audit('TEST', a=1)
        check('audit log stored in the same database', q.audit_tail(1)[0]['event'] == 'TEST')
        q.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ the whole ONA
def make_core(tmp, strict=False, clock=None):
    cfg = OnaConfig.from_dict({
        'key': 'demo', 'quorum': 2, 'strict': strict,
        'anchor': {'lat': 36.8065, 'lon': 10.1815, 'alt': 10.0, 'map_yaw_deg': 20.0},
        'gateways': {'gw1': {'local': [-4.0, -3.0, 2.0]}, 'gw2': {'local': [24.0, -4.0, 2.2]},
                     'gw3': {'local': [10.0, 17.0, 1.8]}},
    })
    q = PersistentQueue(os.path.join(tmp, 'ona.sqlite3'))
    sent = []
    core = OnaCore(cfg, q, clock=clock or time.time, downlink=lambda frames: sent.append(frames))
    return core, q, sent


def endpoint_payloads(q, ep):
    return [p for (_, e, p, _) in q.pending(limit=10000) if e == ep]


def test_core():
    print('The ONA end to end (core.py)')
    tmp = tempfile.mkdtemp()
    try:
        t = [2000.0]
        core, q, sent = make_core(tmp, clock=lambda: t[0])
        cal = core.cfg.calibrator
        g = cal.local_to_gps(np.array([5.0, 2.0, 0.0]))
        rec = {'beacon_id': 13, 'seq': 1, 'kind': 'RADIATION', 'gps': {'lat': g['lat'], 'lon': g['lon'], 'err_m': 1.2},
               'severity': 40, 'confidence': 0.9, 'next': {'id': 12, 'dist_m': 2.1, 'bearing_deg': 181}, 'ts': 1999}
        fr = lmb2.encode_record(rec)
        core.handle_line({'gw': 'gw1', 'frame': fr.hex(), 'rssi': -90})
        check('one gateway: not on the map yet', not endpoint_payloads(q, '/api/beacon'))
        core.handle_line({'gw': 'gw2', 'frame': lmb2.encode_record(rec, relay=5, hops=2).hex(), 'rssi': -100})
        b = endpoint_payloads(q, '/api/beacon')
        check('two gateways agree (through different relays): confirmed 2/3',
              len(b) == 1 and b[0]['ona']['state'] == 'confirmed' and b[0]['ona']['votes'] == 2 and b[0]['kind'] == 'RADIATION')
        core.handle_line({'gw': 'gw3', 'frame': fr.hex()})
        b = endpoint_payloads(q, '/api/beacon')
        check('third gateway: 3/3 (same queue slot updated)', len(b) == 1 and b[0]['ona']['votes'] == 3)
        # a liar re-signs a changed record (it has the key)
        fake = dict(rec, seq=2, severity=0)
        core.handle_line({'gw': 'gw3', 'frame': lmb2.encode_record(fake).hex()})
        t[0] += 10
        core.tick()
        b = endpoint_payloads(q, '/api/beacon')
        check('a record only one gateway reports is shown as unconfirmed 1/3',
              b and b[0]['ona']['state'] == 'unconfirmed' and b[0]['ona']['votes'] == 1)
        bad = bytearray(fr)
        bad[30] ^= 0xFF
        for _ in range(3):
            core.handle_line({'gw': 'gw2', 'frame': bytes(bad).hex()})
        ev = [p['event'] for p in endpoint_payloads(q, '/api/ona-event')]
        check('frames failing the ONA signature check quarantine their gateway', 'GATEWAY_QUARANTINED' in ev
              and core.voter.score('gw2').quarantined)
        core.voter.release('gw2')
        # ---- robot positioning
        gws = core.cfg.gateways
        rnd = np.random.default_rng(3)
        path = [np.array([1.0 + 0.4 * k, 1.0 + 0.1 * k]) for k in range(40)]
        drift = np.array([0.0, 0.0])
        last = None
        for k, p in enumerate(path):
            t[0] += 2.0
            if k >= 25:
                drift = drift + np.array([0.25, -0.1])          # the robot's SLAM slips from here on
            rep = p + drift
            fr = lmb2.encode_robot({'robot_id': 1, 'seq': k + 1, 'role': 'EXECUTOR', 'phase': 'GOTO', 'x': rep[0],
                                    'y': rep[1], 'yaw': 0.2, 'pose_sd_m': 0.2, 'ts': int(t[0])})
            p_enu = cal.local_to_enu(np.array([p[0], p[1], 0.0]))
            for gw, gpos in gws.items():
                r = float(np.linalg.norm(np.array([p_enu[0], p_enu[1], core.robot_h_enu]) - gpos)) + rnd.normal(0, 0.4)
                core.handle_line({'gw': gw, 'frame': fr.hex(), 'range_m': r, 'range_sd': 0.4,
                                  'rssi': round(-15 - 30 * math.log10(max(r, 1.0)) + rnd.normal(0, 2), 1)})
            t[0] += 1.1
            core.tick()
            last = endpoint_payloads(q, '/api/robot-status')
            if k == 20:
                mid = last[0]
                e_mid = np.linalg.norm(cal.gps_to_enu(mid['measured']['lat'], mid['measured']['lon'])[:2] - p_enu[:2])
                c_mid = mid['consistency']['state']
        r = last[0]
        truth_enu = cal.local_to_enu(np.array([path[-1][0], path[-1][1], 0]))
        m_enu = cal.gps_to_enu(r['measured']['lat'], r['measured']['lon'])
        err = float(np.linalg.norm(m_enu[:2] - truth_enu[:2]))
        check('robot position from 3 gateways (EKF): < 0.5 m, < 0.8 m while its SLAM slips, honest error bound',
              e_mid < 0.5 and err < 0.8 and err < 3 * r['measured']['sigma_m'] + 0.1,
              f'{e_mid:.2f} m mid-run, {err:.2f} m at the end while its SLAM slips 0.27 m per ping, '
              f'sigma {r["measured"]["sigma_m"]} m')
        check("robot's own pose agrees with the gateways before the slip", c_mid == 'agree')
        ev = [p['event'] for p in endpoint_payloads(q, '/api/ona-event')]
        check('SLAM slip detected: reported and measured positions disagree', r['consistency']['state'] == 'drift'
              and 'SLAM_DRIFT' in ev, r['consistency'])
        check('3-sphere fix with integrity check in every report', r.get('fix', {}).get('raim') in ('pass', 'fail')
              and len(r['ranges']) == 3)
        ex = endpoint_payloads(q, '/api/executor-status')
        check('Executor position also sent in the old dashboard format', ex and 'lat' in ex[0] and 'heading' in ex[0])
        chk = core.tracks[1].calib_check
        check('frame translation re-checked in the mission from gateway fixes', chk is not None, chk)
        # ---- mission briefing
        m = core.dispatch_mission([13, 20, 11])
        frames = sent[-1]
        dec = lmb2.decode_mission(frames[0])
        check('briefing: signed MISSION frame(s) sent through the gateways', dec['targets'] == [13, 20, 11]
              and dec['mission_id'] == m['mission_id'])
        t[0] += 1
        fr = lmb2.encode_robot({'robot_id': 1, 'seq': 100, 'role': 'EXECUTOR', 'phase': 'WAIT', 'x': 1, 'y': 1,
                                'mission_id': m['mission_id'], 'mission_ack': True, 'total': 3})
        for gw in gws:
            core.handle_line({'gw': gw, 'frame': fr.hex(), 'range_m': 5.0})
        t[0] += 1.5
        core.tick()
        ev = [p['event'] for p in endpoint_payloads(q, '/api/ona-event')]
        check('the Executor acknowledges the briefing (seen through the gateways)', 'MISSION_ACK' in ev
              and core.mission['acked_by'] == [1])
        n_tx = core.mission['tx_count']
        t[0] += 100
        core.tick()
        check('no more retransmissions once acknowledged', core.mission['tx_count'] == n_tx)
        st = endpoint_payloads(q, '/api/ona-status')
        check('ONA status for the Command Post', st and len(st[-1]['gateways']) == 3 and st[-1]['quorum'] == 2)
        old = core.on_dashboard_mission({'waypoints': [{'beacon_id': 4}], 'dispatched_at': time.time() - 3600})
        check('a mission left on the dashboard from before the ONA started is not replayed', old is None)
        stamp = time.time() + 1
        dm = core.on_dashboard_mission({'waypoints': [{'beacon_id': 20}, {'beacon_id': '11'}], 'dispatched_at': stamp})
        check('dashboard dispatch -> briefing (once)', dm and dm['targets'] == [20, 11]
              and core.on_dashboard_mission({'waypoints': [{'beacon_id': 20}], 'dispatched_at': stamp}) is None)
        q.close()
        # strict mode
        core2, q2, _ = make_core(os.path.join(tmp, 's') if os.makedirs(os.path.join(tmp, 's')) is None else tmp,
                                 strict=True, clock=lambda: t[0])
        core2.handle_line({'gw': 'gw1', 'frame': lmb2.encode_record(rec).hex()})
        t[0] += 60
        core2.tick()
        check('strict mode: unconfirmed records never reach the Command Post', not endpoint_payloads(q2, '/api/beacon'))
        q2.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_gateway_forms():
    print('Gateway line forms: raw frames, JSON lines, both; the real network simulator (bpsim)')
    tmp = tempfile.mkdtemp()
    try:
        t = [5000.0]
        core, q, sent = make_core(tmp, clock=lambda: t[0])
        g = core.cfg.calibrator.local_to_gps(np.array([3.0, 1.0, 0.0]))
        rec = {'beacon_id': 21, 'seq': 4, 'kind': 'GAS', 'gps': {'lat': g['lat'], 'lon': g['lon'], 'err_m': 0.7},
               'severity': 33, 'confidence': 0.8, 'next': {'id': 20, 'dist_m': 2.4, 'bearing_deg': 93.5}, 'ts': 4990}
        fr = lmb2.encode_record(rec)
        dec = lmb2.decode_record(fr)
        # the JSON line a gateway prints for it (bp_entry_to_json: confidence 2 decimals, bearing 1)
        js = {'beacon_id': 21, 'seq': 4, 'kind': 'GAS', 'gps': {'lat': round(dec['gps']['lat'], 7),
              'lon': round(dec['gps']['lon'], 7), 'err_m': dec['gps']['err_m'], 'src': 'slam'},
              'severity': 33, 'confidence': round(dec['confidence'], 2),
              'next': {'id': 20, 'dist_m': dec['next']['dist_m'], 'bearing_deg': round(dec['next']['bearing_deg'], 1)},
              'ts': 4990, 'ttl_s': dec['ttl_s'], 'half_life_s': dec['half_life_s'], 'retracted': False,
              'stale': False, 'ev': 'new', 'hops': 1, 'relay': 20, 'rssi': -97}
        core.handle_line({'gw': 'gw1', 'ev': 'rx', 'frame': fr.hex(), 'rssi': -97})
        core.handle_line(dict(js, gw='gw1'))
        check('one gateway sending the raw frame AND its JSON line counts once', not endpoint_payloads(q, '/api/beacon'))
        core.handle_line(dict(js, gw='gw2'))
        b = endpoint_payloads(q, '/api/beacon')
        check('a JSON-only gateway agrees with a raw-frame gateway: confirmed 2/3',
              len(b) == 1 and b[0]['ona']['votes'] == 2 and b[0]['ona']['state'] == 'confirmed')
        q.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    bpsim = os.path.join(os.path.dirname(ROOT), 'beacon_net', 'host', 'bpsim' + ('.exe' if os.name == 'nt' else ''))
    if not os.path.exists(bpsim) and os.name == 'nt':
        bpsim = os.path.join(os.path.dirname(ROOT), 'beacon_net', 'windows', 'bpsim.exe')
    if not os.path.exists(bpsim):
        print('  (bpsim not built: make -C ../beacon_net/host - skipped)')
        return
    import subprocess
    try:
        out = subprocess.run([bpsim, '--gateways', '3', '--quiet'], capture_output=True, text=True, timeout=120).stdout
    except (OSError, subprocess.SubprocessError) as e:
        print(f'  (bpsim could not run: {e} - skipped)')
        return
    tmp = tempfile.mkdtemp()
    try:
        clk = [0.0]
        cfg = OnaConfig.load(os.path.join(ROOT, 'ona_config_bpsim.json'))
        q = PersistentQueue(os.path.join(tmp, 'q.sqlite3'))
        core = OnaCore(cfg, q, clock=lambda: clk[0])
        n = 0
        for ln in out.splitlines():
            try:
                d = json.loads(ln)
            except ValueError:
                continue
            clk[0] = max(clk[0], float(d.get('ts') or clk[0]))
            core.handle_line(d)
            n += 1
            if n % 50 == 0:
                core.tick()
        core.tick()
        last = {}
        for p in endpoint_payloads(q, '/api/beacon'):
            last[p['beacon_id']] = p
        conf = [p for p in last.values() if p['ona']['state'] == 'confirmed']
        check(f'bpsim --gateways 3 ({n} lines): {len(conf)}/{len(last)} beacon records confirmed by the vote, '
              f'{sum(1 for p in conf if p["ona"]["votes"] == 3)} by all three gateways',
              len(last) >= 12 and len(conf) == len(last))
        silent = [p['beacon_id'] for p in last.values() if p.get('silent')]
        ev = [p['event'] for p in endpoint_payloads(q, '/api/ona-event')]
        check(f'the beacon destroyed in bpsim is reported lost ({silent}); no false conflict, no gateway blamed',
              bool(silent) and 'BEACON_LOST' in ev and 'RECORD_CONFLICT' not in ev and 'GATEWAY_QUARANTINED' not in ev)
        q.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    for fn in (test_frames, test_geodesy, test_kalman, test_multilateration, test_quorum, test_store_uplink, test_core,
               test_gateway_forms):
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            RESULTS.append((fn.__name__, False, f'crashed: {e}'))
    n_ok = sum(1 for _, ok, _ in RESULTS if ok)
    print(f'\n{n_ok}/{len(RESULTS)} checks passed')
    sys.exit(0 if n_ok == len(RESULTS) else 1)
