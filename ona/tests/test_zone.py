#!/usr/bin/env python3
"""Whole missions through the ONA: zone simulator -> 3 gateways -> OnaCore (in-process, simulated time).

    python3 tests/test_zone.py
"""
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from ona.core import OnaConfig, OnaCore  # noqa: E402
from ona.store import PersistentQueue  # noqa: E402
from ona.zonesim import ZoneSim, builtin_scenario, walls_crossed  # noqa: E402
import json  # noqa: E402

RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond), detail))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{('  (' + str(detail) + ')') if detail else ''}")


def run(minutes=6.0, faults=None, dispatch=None, seed=1, wait_briefing=True, strict=False, tracker=None):
    tmp = tempfile.mkdtemp()
    with open(os.path.join(os.path.dirname(HERE), 'ona_config.json')) as f:
        cfg_d = json.load(f)
    cfg_d['strict'] = strict
    cfg_d.setdefault('tracker', {}).update(tracker or {})
    cfg = OnaConfig.from_dict(cfg_d)
    q = PersistentQueue(os.path.join(tmp, 'q.sqlite3'))
    box = {}
    sim_ref = {}
    core = OnaCore(cfg, q, clock=lambda: (sim_ref['sim'].t0 + sim_ref['sim'].t) if 'sim' in sim_ref else 1790000000.0,
                   downlink=lambda frames: [sim_ref['sim'].receive_downlink(fr.hex()) for fr in frames])
    posts = []
    sim = ZoneSim(cfg, builtin_scenario(), seed=seed, sink=lambda d: core.handle_line(d), faults=faults or {},
                  wait_briefing=wait_briefing, t0=1790000000.0, log=None)
    sim_ref['sim'] = sim
    sim.start()
    errors = []

    def poll():
        core.tick()
        if dispatch and not box.get('sent') and sim.t >= dispatch[0]:
            box['sent'] = True
            core.dispatch_mission(dispatch[1])
        # everything the ONA queued for the Command Post
        for row_id, ep, p, prio in q.pending(limit=500):
            posts.append((sim.t, ep, p))
            q.mark_sent(row_id, 'LTE')
            if ep == '/api/robot-status' and p.get('seq') in sim.truth and 'measured' in p:
                tr = sim.truth[p['seq']]
                enu = cfg.calibrator.gps_to_enu(p['measured']['lat'], p['measured']['lon'])
                loc = cfg.calibrator.enu_to_local(enu)
                errors.append((sim.t, math.hypot(loc[0] - tr[0], loc[1] - tr[1]), p['measured']['sigma_m'],
                               p.get('consistency', {}).get('state')))
    sim.step_until(minutes * 60, poll=poll)
    out = {'core': core, 'sim': sim, 'posts': posts, 'errors': errors, 'q': q}
    q.close()
    shutil.rmtree(tmp, ignore_errors=True)
    return out


def events(r):
    return [p['event'] for t, ep, p in r['posts'] if ep == '/api/ona-event']


def beacons(r):
    last = {}
    for t, ep, p in r['posts']:
        if ep == '/api/beacon':
            last[p['beacon_id']] = p
    return last


def test_nominal():
    print('Nominal mission: 16 beacons, 3 gateways, Executor briefed from the Command Post')
    r = run(minutes=7.0, dispatch=(8.0, [8, 4, 14]))
    b = beacons(r)
    conf = [p for p in b.values() if p['ona']['state'] == 'confirmed']
    check('every beacon record confirmed by at least 2 gateways', len(conf) == 16 and len(b) == 16,
          f"{len(conf)}/16, 3/3 for {sum(1 for p in conf if p['ona']['votes'] == 3)}")
    sim = r['sim']
    check('briefing received and acknowledged', 'MISSION_ACK' in events(r) and sim.mission_id != 0)
    check('the Executor treats the briefed targets in the given order and returns',
          sim.treated == [8, 4, 14] and sim.finished, f'treated {sim.treated}, finished {sim.finished}')
    e = np.array([x[1] for x in r['errors'][5:]])
    s = np.array([x[2] for x in r['errors'][5:]])
    check('robot position measured by the gateways through walls: RMS < 1.3 m', len(e) > 50 and
          float(np.sqrt(np.mean(e ** 2))) < 1.3,
          f'RMS {np.sqrt(np.mean(e ** 2)):.2f} m, 95 % < {np.percentile(e, 95):.2f} m over {len(e)} pings')
    check('error bound is honest (error < 3 sigma in >= 95 % of pings)', np.mean(e < 3 * s + 0.2) >= 0.95,
          f'{100 * np.mean(e < 3 * s + 0.2):.0f} %')
    walls = [walls_crossed(sim.truth[k], g[:2], sim.boxes) for k in list(sim.truth)[::10] for g in sim.gws.values()]
    check('ranges really went through walls (NLOS)', np.mean(walls) >= 1.0, f'mean {np.mean(walls):.1f} walls')
    cons = [x[3] for x in r['errors'][10:]]
    check("robot's own pose agrees with the gateways (no drift)", cons.count('drift') <= max(2, len(cons) // 20),
          f"{cons.count('drift')} drift / {len(cons)}")
    st = [p for t, ep, p in r['posts'] if ep == '/api/ona-status'][-1]
    chk = st['calibration']['in_mission_check']
    check('frame translation re-checked in the mission: calibration holds', chk and chk['verdict'] == 'holds', chk)


def test_faults():
    print('Faults: a lying gateway, a forged victim, a crushed beacon, a gateway down, a SLAM slip')
    r = run(minutes=5.0, wait_briefing=False, faults={'liar': 'gw3', 'forge': {'gw2': 40.0}, 'killed': {6: 60.0},
                                                        'down': {}, 'slip': [], 'corrupt': {}})
    b = beacons(r)
    ev = events(r)
    sc = r['core'].voter.score('gw3')
    check('the lying gateway is named and quarantined', 'GATEWAY_DISAGREES' in ev and sc.quarantined, sc.note)
    haz = [p for p in b.values() if p['kind'] not in ('WAYPOINT', 'EXIT')]
    sim = r['sim']
    haz = [p for p in haz if p['beacon_id'] in sim.beacons]
    ok = all(p['severity'] == sim.beacons[p['beacon_id']].record['severity'] for p in haz)
    check("the Command Post shows the true hazards, not the liar's version", ok and len(haz) == 4)
    f = b.get(900)
    check('a forged VICTIM from one gateway is never confirmed', f is not None and f['ona']['state'] == 'unconfirmed',
          None if f is None else f['ona'])
    check('a crushed beacon is reported lost, its record kept', 'BEACON_LOST' in ev and b[6]['silent'])
    e2 = [x for x in r['errors'] if x[0] > 90]
    hon = 100.0 * sum(1 for x in e2 if x[1] < 3 * x[2]) / max(1, len(e2))
    check('two gateways left (gw3 ignored): no false SLAM drift, error bars still honest',
          'SLAM_DRIFT' not in ev and hon >= 95.0, f'{hon:.0f} % of {len(e2)} pings within 3 sigma')
    r = run(minutes=6.0, wait_briefing=False, seed=4, faults={'down': {'gw1': [(60.0, 140.0)]},
                                                               'slip': [(170.0, (2.5, -1.5))]})
    ev = events(r)
    down = [x for x in r['errors'] if 70 < x[0] < 135]
    e = np.array([x[1] for x in down])
    check('gateway down: tracking continues with 2 gateways + odometry (tight coupling)',
          len(e) > 10 and float(np.max(e)) < 2.5, f'max {np.max(e):.2f} m over {len(e)} pings')
    check('SLAM slip of 2.9 m detected by the gateways', 'SLAM_DRIFT' in ev)
    check('the drift alert is raised once and does not flap (the slipped pose never came back)',
          ev.count('SLAM_DRIFT') == 1 and 'SLAM_OK' not in ev,
          f"{ev.count('SLAM_DRIFT')} drift / {ev.count('SLAM_OK')} ok")
    chk = r['core'].tracks[1].calib_check if 1 in r['core'].tracks else None
    check('the slip is not blamed on the entrance calibration (frame check still "holds")',
          chk is not None and chk['verdict'] == 'holds', chk and {k: chk[k] for k in ('pairs', 'mean_shift_m', 'verdict')})
    after = [x for x in r['errors'] if x[0] > 200]
    ea = np.array([x[1] for x in after])
    check('after the slip, the measured position still follows the true robot', len(ea) > 10
          and float(np.sqrt(np.mean(ea ** 2))) < 1.5, f'RMS {np.sqrt(np.mean(ea ** 2)):.2f} m')


def test_corrupt_strict():
    print('A gateway passing corrupted frames; strict mode')
    r = run(minutes=3.0, wait_briefing=False, strict=True, faults={'corrupt': {'gw2': 10.0}, 'forge': {'gw1': 20.0}})
    ev = events(r)
    check('bad signatures from a gateway get it quarantined', 'GATEWAY_QUARANTINED' in ev
          and r['core'].voter.score('gw2').quarantined)
    b = beacons(r)
    check('strict mode: nothing unconfirmed on the map (no forged victim)', 900 not in b
          and all(p['ona']['state'] == 'confirmed' for p in b.values()), f'{len(b)} beacons shown')


if __name__ == '__main__':
    for fn in (test_nominal, test_faults, test_corrupt_strict):
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            RESULTS.append((fn.__name__, False, f'crashed: {e}'))
    n_ok = sum(1 for _, ok, _ in RESULTS if ok)
    print(f'\n{n_ok}/{len(RESULTS)} checks passed')
    sys.exit(0 if n_ok == len(RESULTS) else 1)
