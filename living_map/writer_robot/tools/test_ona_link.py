#!/usr/bin/env python3
"""
test_ona_link.py - the robots and the Outside Network Area (ONA) together, without ROS.

  1. the robots' copy of the ONA frame codec is the ONA's own (byte for byte)
  2. the gateway radio model in the Gazebo worlds: walls, ranges, link budget
  3. the WHOLE chain in-process, in the 2D simulator on the big map:
       executor_node + beacon_radio_sim + ona_link_node  ->  the real ONA (OnaCore)
     - every beacon record reaches the ONA through the three gateways and is
       confirmed by at least two of them
     - the Command Post dispatches a briefing (#25 then #11): the ONA signs it,
       the gateways transmit it, ona_link_node checks it and the Executor
       treats exactly those, in that order, and acknowledges it in its pings
     - the ONA measures the Executor's position from the three distances,
       through the walls, and the robot's own pose agrees with it
     - a SLAM slip injected in the robot's own pose is caught by the ONA

    python3 tools/test_ona_link.py
"""
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, 'ros_stub'))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import rclpy  # noqa: E402  (the stub)
from rclpy import node as bus  # noqa: E402
import tf2_ros  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402

RESULTS = []


def check(ok, text):
    RESULTS.append((bool(ok), text))
    print(f'  [{"PASS" if ok else "FAIL"}] {text}')


def find_ona():
    """The ONA package: copied into this package by install.sh (ona/), or next to it (living_map/ona)."""
    for d in (os.path.join(ROOT, 'ona'), os.path.join(os.path.dirname(ROOT), 'ona'),
              os.path.expanduser('~/living_map/ona')):
        if os.path.exists(os.path.join(d, 'ona', 'core.py')):
            return d
    return None


# ====================================================================== 1
def test_codec(ona_dir):
    print('\n== frame codec: the robots\' copy and the ONA\'s')
    from writer_robot import ona_lmb2
    mine = open(ona_lmb2.__file__, encoding='utf-8').read()
    body = mine[mine.index('"""'):]
    theirs = open(os.path.join(ona_dir, 'ona', 'lmb2.py'), encoding='utf-8').read()
    check(body == theirs, 'writer_robot/ona_lmb2.py is the ONA\'s lmb2.py (same bytes after the header)')
    fr = ona_lmb2.encode_robot({'robot_id': 1, 'seq': 7, 'role': 'EXECUTOR', 'phase': 'TREAT', 'x': 3.25,
                                'y': -1.5, 'yaw': 1.0, 'mission_id': 77, 'mission_ack': True, 'done': 1, 'total': 2})
    d = ona_lmb2.decode_robot(fr)
    check(len(fr) == 44 and d['phase'] == 'TREAT' and d['mission_ack'] and abs(d['x'] - 3.25) < 1e-3,
          'a 44-byte ROBOT frame round-trips (phase, briefing ack, position in mm)')
    bad = bytearray(fr)
    bad[14] ^= 1
    try:
        ona_lmb2.decode_robot(bytes(bad))
        ok = False
    except ona_lmb2.DecodeError:
        ok = True
    check(ok, 'a ROBOT frame with one flipped bit is rejected (HMAC)')


# ====================================================================== 2
def test_radio():
    print('\n== the virtual gateways around the Gazebo buildings')
    from writer_robot.ona_radio import GatewayRadio, load_world_boxes, load_config
    for world, cfg_name in (('contaminated_zone.world', 'ona_gazebo_big.json'),
                            ('retreat_test.world', 'ona_gazebo_retreat.json')):
        boxes = load_world_boxes(os.path.join(ROOT, 'worlds', world))
        cfg = load_config(os.path.join(ROOT, 'config', cfg_name))
        radio = GatewayRadio(cfg['gateways'], boxes, seed=3)
        xs = np.linspace(min(b[0] for b in boxes) + 0.5, max(b[0] for b in boxes) - 0.5, 9)
        ys = np.linspace(min(b[1] for b in boxes) + 0.5, max(b[1] for b in boxes) - 0.5, 7)
        heard, longer, walls = [], [], []
        for x in xs:
            for y in ys:
                lines = radio.ping_lines(b'\x23' * 44, (x, y))
                heard.append(len(lines))
                for ln in lines:
                    g = cfg['gateways'][ln['gw']]
                    true_r = math.sqrt((x - g[0]) ** 2 + (y - g[1]) ** 2 + (0.3 - g[2]) ** 2)
                    nw = len(radio.walls((x, y), g[:2]))
                    walls.append(nw)
                    if nw:
                        longer.append(ln['range_m'] - true_r)
        check(len(boxes) >= 10 and min(heard) == 3,
              f'{world}: {len(boxes)} walls; all 3 gateways hear the robot everywhere inside ({len(heard)} spots)')
        check(np.mean(walls) > 1.0 and np.mean(longer) > 0.2,
              f'{world}: on average {np.mean(walls):.1f} walls in the way, which make the distances '
              f'{np.mean(longer):.2f} m too long (the ONA corrects it from the signal loss)')


# ====================================================================== 3
def test_chain(ona_dir, minutes=8.0, slip_at=None):
    import sim2d
    import sim_mission
    import importlib
    import subprocess
    sys.path.insert(0, ona_dir)
    from ona.core import OnaConfig, OnaCore
    from ona.store import PersistentQueue
    import writer_robot.executor_node as exn
    import writer_robot.beacon_radio_sim as brs
    import writer_robot.ona_link_node as oln
    for m in (exn, brs, oln):
        importlib.reload(m)
    title = 'executor_node + ona_link_node -> the ONA (3 gateways) -> briefing -> Executor'
    if slip_at:
        title += f', SLAM slip at {slip_at:.0f} s'
    print(f'\n== {title}')
    world = sim_mission.prepare_world(os.path.join(ROOT, 'worlds', 'contaminated_zone.world'))
    mfile = os.path.join(ROOT, 'missions', 'demo_big.json')
    cfg_file = os.path.join(ROOT, 'config', 'ona_gazebo_big.json')
    tmp = tempfile.mkdtemp()
    EPOCH = 1790000000.0
    cfg = OnaConfig.load(cfg_file)
    q = PersistentQueue(os.path.join(tmp, 'q.sqlite3'))
    downlink = []
    core = OnaCore(cfg, q, clock=lambda: EPOCH + bus.CLOCK['t'],
                   downlink=lambda frames: downlink.extend({'ev': 'tx', 'gw': g, 'frame': f.hex()}
                                                           for f in frames for g in cfg.gateways))

    class InProcess:
        sent = 0

        def send(self, d):
            InProcess.sent += 1
            core.handle_line(dict(d))

        def poll(self):
            out = list(downlink)
            downlink.clear()
            return out

    spawned = []

    class FakePopen:
        def __init__(self, cmd, **kw):
            spawned.append(cmd)
    old_which, old_popen = shutil.which, subprocess.Popen
    shutil.which = lambda name: '/usr/bin/' + name
    subprocess.Popen = FakePopen
    posts, errs, cons = [], [], []
    slip = np.zeros(2)
    try:
        bus.reset()
        bus.PARAM_OVERRIDES.update({'mission_file': mfile, 'robot': 'executor', 'ona_config': cfg_file,
                                    'world_file': os.path.join(ROOT, 'worlds', 'contaminated_zone.world'),
                                    'wait_briefing': 30.0, 'retime': False, 'advertise_speed': False})
        robot = sim_mission.Robot(world, np.random.default_rng(7))
        tf2_ros.POSE[('map', 'base_footprint')] = (0.0, 0.0, 0.0)
        brs.BeaconRadioSim()
        node = exn.ExecutorNode()
        link = oln.OnaLinkNode(transport=InProcess())
        dt, t = 0.05, 0.0
        nxt = {'scan': 0.0, 'map': 0.0, 'cam': 0.0, 'odom': 0.0, 'ona': 0.0}
        angles = robot.angles
        dispatched = None
        while t < minutes * 60:
            x, y, yaw = robot.x, robot.y, robot.yaw
            if slip_at and t >= slip_at:
                slip = np.array([1.8, -1.2])            # the robot's SLAM jumps 2.2 m (a bad loop closure)
            tf2_ros.POSE[('map', 'base_footprint')] = (x + slip[0], y + slip[1], yaw)
            if t >= nxt['odom']:
                nxt['odom'] += 0.1
                o = Odometry()
                o.pose.pose.position.x, o.pose.pose.position.y = x, y
                bus._Publisher('/odom').publish(o)
            if t >= nxt['scan']:
                nxt['scan'] += 1 / 8.0
                lx, ly = x + 0.10 * math.cos(yaw), y + 0.10 * math.sin(yaw)
                r = sim2d.cast(world.segs, lx, ly, angles + yaw, 12.0)
                robot.mapper.update(lx, ly, angles + yaw, r, 10.0, heading=yaw, stamp=t)
                m = LaserScan()
                m.angle_min, m.angle_max, m.angle_increment = -math.pi, math.pi, angles[1] - angles[0]
                m.range_min, m.range_max = 0.15, 12.0
                m.ranges = [float(v) if np.isfinite(v) else float('inf') for v in r]
                bus._Publisher('/scan').publish(m)
            if t >= nxt['map']:
                nxt['map'] += 2.0
                g = robot.mapper.grid()
                ki, kj = np.nonzero(g >= 0)
                i0, i1 = max(ki.min() - 20, 0), min(ki.max() + 21, g.shape[0])
                j0, j1 = max(kj.min() - 20, 0), min(kj.max() + 21, g.shape[1])
                sub = g[i0:i1, j0:j1]
                m = OccupancyGrid()
                m.info.resolution = robot.mapper.res
                m.info.width, m.info.height = sub.shape[1], sub.shape[0]
                m.info.origin.position.x = robot.mapper.ox + j0 * robot.mapper.res
                m.info.origin.position.y = robot.mapper.oy + i0 * robot.mapper.res
                m.data = sub.ravel().astype(int).tolist()
                bus._Publisher('/map').publish(m)
            if t >= nxt['cam']:
                nxt['cam'] += 1 / 6.0
                for kind, ex, ey, depth in sim2d.camera_detect(world, x, y, yaw, robot.rng):
                    bus._Publisher('/executor/hazard').publish(
                        String(json.dumps({'type': kind, 'x': ex, 'y': ey, 'range': depth, 'bearing': 0.0})))
            # the Command Post operator dispatches a mission once the records are on the dashboard
            if dispatched is None and t >= 12.0:
                dispatched = core.dispatch_mission([25, 11])
            if t >= nxt['ona']:
                nxt['ona'] += 0.5
                core.tick()
                for row_id, ep, p, prio in q.pending(limit=500):
                    posts.append((t, ep, p))
                    q.mark_sent(row_id, 'LTE')
                    if ep == '/api/robot-status' and 'measured' in p:
                        enu = cfg.calibrator.gps_to_enu(p['measured']['lat'], p['measured']['lon'])
                        loc = cfg.calibrator.enu_to_local(enu)
                        errs.append((t, math.hypot(loc[0] - robot.x, loc[1] - robot.y), p['measured']['sigma_m']))
                        cons.append((t, (p.get('consistency') or {}).get('state')))
            bus.run_timers(t)
            cmds = bus.PUBLISHED.get('/cmd_vel', [])
            cmd = (cmds[-1].linear.x, cmds[-1].angular.z) if cmds else (0.0, 0.0)
            robot.move(cmd, dt)
            t += dt
            if node.core.phase == 'DONE' and t - node.core.t_state > 4.0:
                break
            for k in ('/cmd_vel', '/executor/status', '/scan', '/executor/telemetry', '/odom', '/executor/lora_rx'):
                if len(bus.PUBLISHED.get(k, [])) > 3000:
                    bus.PUBLISHED[k] = bus.PUBLISHED[k][-5:]
        c = node.core
        print(f'     finished in {t / 60:.1f} simulated min, phase {c.phase}, treated '
              + ' -> '.join(f'#{d["id"]}' for d in c.done_targets) + f'; {InProcess.sent} gateway lines')
        recs = [p for _, ep, p in posts if ep == '/api/beacon']
        final = {}
        for p in recs:
            final[p['beacon_id']] = p
        n_mission = len(json.load(open(mfile))['beacons'])
        conf = [p for p in final.values() if p.get('ona', {}).get('state') == 'confirmed']
        check(len(final) == n_mission and len(conf) == n_mission,
              f'all {n_mission} beacon records reached the Command Post, each confirmed by >= 2 of the 3 '
              f'gateways ({len(conf)} confirmed; {sum(1 for p in conf if p["ona"]["votes"] == 3)} by all three)')
        br = bus.PUBLISHED.get('/executor/briefing', [])
        check(len(br) == 1 and json.loads(br[0].data)['targets'] == [25, 11],
              f'briefing heard over the air, signature checked, published once on /executor/briefing '
              f'({len(br)} message(s))')
        if not slip_at:
            got = [d['id'] for d in c.done_targets]
            check(got == [25, 11] and c.phase == 'DONE' and math.hypot(robot.x, robot.y) < 0.7,
                  f'the Executor treated exactly the briefed targets, in order ({got}), and came back')
        acked = core.mission and 1 in core.mission.get('acked_by', [])
        evs = [p['event'] for _, ep, p in posts if ep == '/api/ona-event']
        check(acked and 'MISSION_ACK' in evs and 'MISSION_DISPATCH' in evs,
              f'the ONA got the acknowledgement in the robot\'s pings (events: '
              f'{", ".join(sorted(set(evs)))})')
        e = np.array([x[1] for x in errs if not slip_at or x[0] < slip_at or x[0] > slip_at + 20])
        if len(e):
            check(len(e) > 30 and float(np.sqrt(np.mean(e ** 2))) < 1.3,
                  f'the ONA measured the moving Executor through the walls: RMS {np.sqrt(np.mean(e ** 2)):.2f} m, '
                  f'95 % < {np.percentile(e, 95):.2f} m over {len(e)} pings')
        else:
            check(False, 'the ONA measured the Executor (no robot-status)')
        if not slip_at:
            n_drift = sum(1 for _, s in cons if s == 'drift')
            check(n_drift == 0, f'the robot\'s own pose agrees with the gateways ({n_drift} drift / {len(cons)})')
        else:
            after = [s for tt, s in cons if tt > slip_at + 15]
            check('SLAM_DRIFT' in evs and after and after.count('drift') > 0.8 * len(after),
                  f'the slip of the robot\'s own pose is caught by the gateways: SLAM_DRIFT, '
                  f'{after.count("drift")}/{len(after)} later pings flagged')
    finally:
        shutil.which, subprocess.Popen = old_which, old_popen
        q.close()
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    rclpy.init()
    ona_dir = find_ona()
    if ona_dir is None:
        print('the ONA package was not found (ona/ in this package, or ../ona): '
              'only the robot-side tests run')
    else:
        test_codec(ona_dir)
    test_radio()
    if ona_dir is not None:
        test_chain(ona_dir)
        test_chain(ona_dir, minutes=4.0, slip_at=60.0)
    ok = all(r[0] for r in RESULTS)
    print(f'\n{sum(r[0] for r in RESULTS)}/{len(RESULTS)} checks passed - {"ALL PASSED" if ok else "FAILURES"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
