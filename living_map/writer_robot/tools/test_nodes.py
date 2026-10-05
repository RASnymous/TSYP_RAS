#!/usr/bin/env python3
"""
test_nodes.py - runs the Writer robot's ROS nodes WITHOUT ROS and checks them.

A tiny stand-in for rclpy / tf2 / messages (tools/ros_stub) lets the real node
code run in-process:

  1. vision_event_detector  synthetic RGB-D frames rendered with a pinhole
     camera: purple / red / yellow blocks must be detected and placed at the
     right map position; rubble, walls and dropped beacons must not be.
  2. frontier_explorer      the node drives the 2D simulator (tools/sim2d.py)
     through /map, /scan, TF, /writer/hazard and /cmd_vel until it finishes.
  3. beacon_drop_node + mission_recorder: exit, trail and hazard beacons,
     links back to the exit, no duplicates when driving back along the trail,
     spawn command, mission file with verified 44-byte frames.
  4. executor_node + beacon_radio_sim: the Executor reads the Writer's mission
     (frames only) and drives the 2D simulator to every hazard and back.
  5. mine_sensors (the Gafsa mine): air readings, radon, gas, fire, victim,
     roof, phosphate grade, mineral finds and the victim search, published as
     the vision node would; the records they become (grade, SEARCHED).

    python3 tools/test_nodes.py            (needs numpy + opencv-python)
"""
import json
import math
import os
import sys

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
from sensor_msgs.msg import Image, LaserScan  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402

RESULTS = []


def check(ok, text):
    RESULTS.append((bool(ok), text))
    print(f'  [{"PASS" if ok else "FAIL"}] {text}')


# ====================================================================== vision
W_IMG, H_IMG, HFOV = 320, 240, 1.089
FX = (W_IMG / 2) / math.tan(HFOV / 2)
CAM_X, CAM_Z = 0.213, 0.135


def render(pose, boxes):
    """Pinhole render. boxes: (cx, cy, sx, sy, sz, bgr). Returns (bgr, depth)."""
    x, y, yaw = pose
    cx0, cy0 = x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)
    u, v = np.meshgrid(np.arange(W_IMG) + 0.5, np.arange(H_IMG) + 0.5)
    right = (u - W_IMG / 2) / FX
    down = (v - H_IMG / 2) / FX
    f = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    l = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    d = f[None, None, :] + l[None, None, :] * (-right)[..., None] + np.array([0, 0, 1.0])[None, None, :] * (-down)[..., None]
    o = np.array([cx0, cy0, CAM_Z])
    img = np.zeros((H_IMG, W_IMG, 3), np.float32)
    img[:] = (70, 70, 70)
    depth = np.full((H_IMG, W_IMG), np.inf, np.float32)
    # ground
    tg = np.where(d[..., 2] < -1e-6, -CAM_Z / np.minimum(d[..., 2], -1e-6), np.inf)
    hit = np.isfinite(tg) & (tg < 15)
    img[hit] = (110, 110, 110)
    depth[hit] = tg[hit]
    for (bx, by, sx, sy, sz, bgr) in boxes:
        lo = np.array([bx - sx / 2, by - sy / 2, 0.0])
        hi = np.array([bx + sx / 2, by + sy / 2, sz])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[None, None, :] - o) / d
            t2 = (hi[None, None, :] - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=2)
        tmax = np.nanmin(np.maximum(t1, t2), axis=2)
        hb = (tmax >= tmin) & (tmin > 0) & (tmin < depth)
        # shade by face: which slab gave tmin
        axis = np.argmax(np.minimum(t1, t2), axis=2)
        shade = np.where(axis == 2, 1.0, np.where(axis == 0, 0.85, 0.65))
        col = np.array(bgr, np.float32)
        img[hb] = col[None, :] * shade[hb][:, None]
        depth[hb] = tmin[hb]           # d has forward component 1 -> t = z-depth
    img += np.random.default_rng(0).normal(0, 2.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8), depth


def bgr_of(rgb01, k=1.0):
    r, g, b = rgb01
    return (min(255, 255 * b * k), min(255, 255 * g * k), min(255, 255 * r * k))


def test_vision():
    print('\n== vision_event_detector')
    bus.reset()
    import importlib
    import writer_robot.vision_event_detector as ved
    importlib.reload(ved)
    pose = (2.0, 1.0, 0.5)
    tf2_ros.POSE[('map', 'base_footprint')] = pose
    node = ved.VisionEventDetector()
    cases = [
        ('radiation', (0.9, 0.0, 0.9)),
        ('fire', (0.95, 0.05, 0.05)),
        ('gas', (1.0, 0.85, 0.0)),
    ]
    for dist, lateral in ((2.0, 0.0), (3.0, 0.6), (3.5, -0.5)):
        for kind, rgb in cases:
            bus.PUBLISHED.clear()
            bus.CLOCK['t'] += 1.0
            # block position: `dist` ahead of the camera, `lateral` to the left
            x, y, yaw = pose
            cx = x + (CAM_X + dist) * math.cos(yaw) - lateral * math.sin(yaw)
            cy = y + (CAM_X + dist) * math.sin(yaw) + lateral * math.cos(yaw)
            img, dep = render(pose, [(cx, cy, 0.3, 0.3, 0.6, bgr_of(rgb, 0.95))])
            depth_msg = Image(dep, '32FC1')
            node.on_depth(depth_msg)
            im = Image(img, 'bgr8')
            node.on_image(im)
            hz = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/hazard', [])]
            ok = len(hz) == 1 and hz[0]['type'] == kind
            err = math.hypot(hz[0]['x'] - cx, hz[0]['y'] - cy) if ok else 99
            check(ok and err < 0.30, f'{kind:9s} block {dist:.1f} m ahead, {lateral:+.1f} m left: '
                                     f'found at {err:.2f} m from its centre (front face is 0.15 m)')
    # things that must NOT be hazards
    bus.PUBLISHED.clear()
    x, y, yaw = pose
    ahead = lambda d, lat: (x + d * math.cos(yaw) - lat * math.sin(yaw), y + d * math.sin(yaw) + lat * math.cos(yaw))
    boxes = [
        (*ahead(2.0, 0.4), 0.6, 0.5, 0.5, bgr_of((0.6, 0.5, 0.4))),       # rubble
        (*ahead(1.2, -0.3), 0.14, 0.10, 0.08, bgr_of((0.1, 1.0, 0.3))),   # dropped beacon body
        (*ahead(0.6, 0.0), 0.06, 0.06, 0.12, bgr_of((0.1, 0.3, 1.0))),    # beacon LED, very close
        (*ahead(4.0, 0.0), 0.2, 3.0, 1.0, bgr_of((0.8, 0.8, 0.8))),       # wall
    ]
    img, dep = render(pose, boxes)
    node.on_depth(Image(dep, '32FC1'))
    node.on_image(Image(img, 'bgr8'))
    check(not bus.PUBLISHED.get('/writer/hazard'), 'rubble, wall, dropped beacon (green body, blue LED): no hazard')
    # one beacon event per hazard, 16-bit depth (mm) also works
    bus.reset()
    tf2_ros.POSE[('map', 'base_footprint')] = pose
    node = ved.VisionEventDetector()
    cx, cy = ahead(CAM_X + 2.5, 0.0)
    img, dep = render(pose, [(cx, cy, 0.3, 0.3, 0.6, bgr_of((1.0, 0.85, 0.0), 0.95))])
    dep16 = np.where(np.isfinite(dep), dep * 1000.0, 0).astype(np.uint16)
    for k in range(5):
        bus.CLOCK['t'] = k * 5.0
        node.on_depth(Image(dep16, '16UC1'))
        node.on_image(Image(img, 'bgr8'))
    ev = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/events', [])]
    check(len(ev) == 1 and ev[0]['event_type'] == 'gas' and
          math.hypot(ev[0]['event_x'] - cx, ev[0]['event_y'] - cy) < 0.3,
          f'one beacon event for the gas block (got {len(ev)}), 16-bit depth image')
    check(len(ev) == 1 and abs(ev[0]['x'] - pose[0]) < 1e-6,
          'event x/y = robot position (beacon drop point), event_x/event_y = the hazard')
    # no depth image at all (bridge problem): range from where the block meets the floor
    bus.reset()
    tf2_ros.POSE[('map', 'base_footprint')] = pose
    node = ved.VisionEventDetector()
    for dist, lateral in ((1.5, 0.0), (2.5, 0.4), (3.3, -0.3)):
        bus.PUBLISHED.clear()
        bus.CLOCK['t'] += 1.0
        cx, cy = ahead(CAM_X + dist, lateral)
        img, _ = render(pose, [(cx, cy, 0.3, 0.3, 0.6, bgr_of((0.95, 0.05, 0.05), 0.95))])
        node.on_image(Image(img, 'bgr8'))
        hz = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/hazard', [])]
        err = math.hypot(hz[0]['x'] - cx, hz[0]['y'] - cy) if hz else 99
        check(len(hz) == 1 and hz[0]['type'] == 'fire' and err < 0.35,
              f'no depth image: fire block {dist:.1f} m ahead still found, {err:.2f} m from its centre')
    node._report()
    rep = [m for m in bus.LOG if '[health]' in m[3]]
    check(any('ranged without depth' in m[3] for m in rep) and any('no depth images' in m[3] for m in rep),
          'health log reports the missing depth images')


# ====================================================================== explorer
def test_explorer(world_file, minutes, expect_hazards):
    import sim2d
    import importlib
    import writer_robot.frontier_explorer as fe
    importlib.reload(fe)
    print(f'\n== frontier_explorer node driving the 2D simulator: {os.path.basename(world_file)}')
    bus.reset()
    world = sim2d.World(world_file)
    world.los = {}
    for h in world.hazards:
        segs = []
        for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
            if name != h[6]:
                segs.extend(sim2d.rect_segments(cx, cy, sx, sy, yw))
        world.los[h[6]] = np.array(segs)
    mapper = sim2d.Mapper(world.bounds)
    rng = np.random.default_rng(3)
    x, y, yaw = 0.0, 0.0, 0.0
    tf2_ros.POSE[('map', 'base_footprint')] = (x, y, yaw)
    node = fe.FrontierExplorer()
    angles = np.linspace(-math.pi, math.pi, 180)
    v = w = 0.0
    dt, t = 0.05, 0.0
    nxt = {'scan': 0.0, 'map': 0.0, 'cam': 0.0}
    collisions = 0
    seen = set()
    min_after = {}
    while t < minutes * 60:
        tf2_ros.POSE[('map', 'base_footprint')] = (x, y, yaw)
        if t >= nxt['scan']:
            nxt['scan'] += 1 / 8.0
            lx, ly = x + 0.10 * math.cos(yaw), y + 0.10 * math.sin(yaw)
            r = sim2d.cast(world.segs, lx, ly, angles + yaw, 12.0)
            mapper.update(lx, ly, angles + yaw, r, 10.0, heading=yaw, stamp=t)
            m = LaserScan()
            m.angle_min, m.angle_max = -math.pi, math.pi
            m.angle_increment = angles[1] - angles[0]
            m.range_min, m.range_max = 0.15, 12.0
            m.ranges = [float(q) if np.isfinite(q) else float('inf') for q in r + rng.normal(0, 0.01, r.shape)]
            bus._Publisher('/scan').publish(m)
        if t >= nxt['map']:
            nxt['map'] += 2.0
            g = mapper.grid()
            ki, kj = np.nonzero(g >= 0)
            i0, i1 = max(ki.min() - 20, 0), min(ki.max() + 21, g.shape[0])
            j0, j1 = max(kj.min() - 20, 0), min(kj.max() + 21, g.shape[1])
            sub = g[i0:i1, j0:j1]
            m = OccupancyGrid()
            m.info.resolution = mapper.res
            m.info.width, m.info.height = sub.shape[1], sub.shape[0]
            m.info.origin.position.x = mapper.ox + j0 * mapper.res
            m.info.origin.position.y = mapper.oy + i0 * mapper.res
            m.data = sub.ravel().astype(int).tolist()          # ROS: int8[] as a list
            bus._Publisher('/map').publish(m)
        if t >= nxt['cam']:
            nxt['cam'] += 1 / 6.0
            for kind, ex, ey, depth in sim2d.camera_detect(world, x, y, yaw, rng):
                msg = String()
                msg.data = json.dumps({'type': kind, 'x': ex, 'y': ey, 'range': depth, 'bearing': 0.0})
                bus._Publisher('/writer/hazard').publish(msg)
                for h in world.hazards:
                    if h[0] == kind and math.hypot(h[1] - ex, h[2] - ey) < 1.5:
                        seen.add(h[6])
        tf2_ros.POSE[('map', 'base_footprint')] = (x, y, yaw)
        bus.run_timers(t)
        cmds = bus.PUBLISHED.get('/cmd_vel', [])
        tv, tw = (cmds[-1].linear.x, cmds[-1].angular.z) if cmds else (0.0, 0.0)
        v += max(-2.0 * dt, min(2.0 * dt, tv - v))
        w += max(-3.0 * dt, min(3.0 * dt, tw - w))
        ny = yaw + w * dt
        nx_ = x + v * math.cos(yaw + w * dt / 2) * dt
        ny_ = y + v * math.sin(yaw + w * dt / 2) * dt
        if world.collides(nx_, ny_, ny):
            collisions += 1
            v = w = 0.0
        else:
            x, y, yaw = nx_, ny_, ny
        for h in world.hazards:
            if h[6] in seen:
                min_after[h[6]] = min(min_after.get(h[6], 9e9), math.hypot(h[1] - x, h[2] - y))
        t += dt
        if node.core.state == 'DONE' and t - node.core.t_state > 13.0:
            break
        if len(bus.PUBLISHED.get('/cmd_vel', [])) > 2000:
            bus.PUBLISHED['/cmd_vel'] = bus.PUBLISHED['/cmd_vel'][-5:]
    core = node.core
    print(f'     finished in {t / 60:.1f} simulated min, state {core.state}, '
          f'{len(core.hazards)} hazards, {core.retreat_count} retreats')
    check(collisions == 0, f'no collision ({collisions})')
    check(core.state == 'DONE' and math.hypot(x, y) < 0.6, 'exploration completed, back at the start, stopped')
    check(len(seen) == expect_hazards, f'hazards spotted {len(seen)}/{expect_hazards}')
    check(all(d >= 0.9 for d in min_after.values()), 'kept away from every hazard after seeing it')
    paths = bus.PUBLISHED.get('/writer/plan', [])
    check(any(len(p.poses) >= 2 for p in paths) and all(p.header.frame_id == 'map' for p in paths),
          f'/writer/plan published ({len(paths)} paths, map frame)')
    mk = bus.PUBLISHED.get('/writer/explorer_markers', [])
    ok_mk = any(any(m.ns == 'keepout' and m.type == 3 for m in a.markers) for a in mk)
    check(ok_mk, '/writer/explorer_markers shows the keep-out zones')
    st = bus.PUBLISHED.get('/writer/explorer_status', [])
    check(bool(st) and 'DONE' in st[-1].data, f'/writer/explorer_status ends with: "{st[-1].data if st else ""}"')
    last = bus.PUBLISHED.get('/cmd_vel', [])[-1]
    check(last.linear.x == 0.0 and last.angular.z == 0.0, 'robot commanded to stop at the end')


# ====================================================================== beacons
def test_beacons():
    print('\n== beacon_drop_node + mission_recorder')
    bus.reset()
    import importlib
    import shutil
    import subprocess
    import tempfile
    import writer_robot.beacon_drop_node as bdn
    import writer_robot.mission_recorder as mrec
    from writer_robot.mission_log import load_mission, decode_mission
    importlib.reload(bdn)
    importlib.reload(mrec)
    spawned = []

    class FakePopen:
        def __init__(self, cmd, **kw):
            spawned.append(cmd)
    old_which, old_popen = shutil.which, subprocess.Popen
    shutil.which = lambda name: '/usr/bin/' + name
    subprocess.Popen = FakePopen
    tmp = tempfile.mkdtemp()
    try:
        bus.PARAM_OVERRIDES['mission_dir'] = tmp
        tf2_ros.POSE[('map', 'base_footprint')] = (0.0, 0.0, 0.0)
        bdn.BeaconDropNode()           # kept alive by their timers / subscriptions
        mrec.MissionRecorder()
        # drive: 6 m east, turn left, 4 m north, then back the same way
        route = [(i * 0.1, 0.0, 0.0) for i in range(61)]
        route += [(6.0, j * 0.1, math.pi / 2) for j in range(1, 41)]
        back = [(6.0, 4.0 - j * 0.1, -math.pi / 2) for j in range(1, 41)]
        back += [(6.0 - i * 0.1, 0.0, math.pi) for i in range(1, 61)]
        t = 0.0
        n_out = None
        for k, p in enumerate(route + back):
            if k == len(route):
                n_out = len(bus.PUBLISHED.get('/writer/beacon_dropped', []))
            tf2_ros.POSE[('map', 'base_footprint')] = p
            for _ in range(5):             # 0.5 s per 0.1 m (0.2 m/s)
                t += 0.1
                bus.run_timers(t)
        for _ in range(40):
            t += 0.1
            bus.run_timers(t)
        drops = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/beacon_dropped', [])]
        trail = [d for d in drops if d['event_type'] == 'waypoint']
        print('     beacons:', ', '.join(f'#{d["beacon_id"]} {d["event_type"][:4]} ({d["x"]:.1f},{d["y"]:.1f})'
                                        f'->{d["link_id"]}' for d in drops))
        check(drops and drops[0]['event_type'] == 'exit' and drops[0]['link_id'] is None
              and abs(drops[0]['x'] + 0.12) < 1e-6, 'EXIT beacon #0 dropped at the start, under the magazine')
        check(3 <= n_out - 1 <= 8, f'{n_out - 1} trail beacons on the way out (10 m, one turn)')
        pts = [(d['x'], d['y']) for d in drops[:n_out]]
        gaps = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(pts[:-1], pts[1:])]
        check(gaps and max(gaps) <= 2.5 + 0.15, f'trail gaps <= 2.5 m + one step (max {max(gaps) if gaps else 0:.2f} m)')
        corner = any(abs(d['x'] - 6.0 + 0.0) < 0.2 and d['y'] < 0.6 for d in trail[:n_out])
        check(corner, 'a trail beacon marks the corner')
        close = min((math.hypot(a['x'] - b['x'], a['y'] - b['y']) for i, a in enumerate(trail)
                     for b in trail[i + 1:]), default=9.0)
        check(close >= 0.8, f'driving back along the trail: no beacon next to another '
                            f'(closest pair {close:.2f} m, {len(trail) - (n_out - 1)} new on the way back)')
        # hazard event, robot standing at (3, 0) facing +x
        tf2_ros.POSE[('map', 'base_footprint')] = (3.0, 0.0, 0.0)
        ev = {'event_type': 'gas', 'value': 0.2, 'x': 3.0, 'y': 0.0, 'theta': 0.0,
              'event_x': 5.4, 'event_y': 0.3, 'range_m': 2.4, 'bearing_rad': 0.1, 'timestamp': 1.0}
        bus._Publisher('/writer/events').publish(String(json.dumps(ev)))
        for _ in range(30):
            t += 0.1
            bus.run_timers(t)
        drops = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/beacon_dropped', [])]
        gas = [d for d in drops if d['event_type'] == 'gas']
        check(len(gas) == 1 and gas[0]['event_x'] == 5.4 and gas[0]['event_y'] == 0.3,
              'gas event -> gas beacon carrying the hazard position')
        ids = {d['beacon_id']: d for d in drops}
        ok_tree, max_leg = True, 0.0
        for d in drops:
            seen, b = set(), d
            while b['link_id'] is not None:
                if b['link_id'] not in ids or b['link_id'] >= b['beacon_id'] or b['beacon_id'] in seen:
                    ok_tree = False
                    break
                seen.add(b['beacon_id'])
                p_ = ids[b['link_id']]
                max_leg = max(max_leg, math.hypot(b['x'] - p_['x'], b['y'] - p_['y']))
                b = p_
            ok_tree = ok_tree and b['event_type'] == 'exit'
        check(ok_tree and max_leg < 3.8, f'every beacon links back to the EXIT through older beacons '
                                         f'(longest link {max_leg:.2f} m)')
        cmd = spawned[-1]
        xi, zi = cmd.index('-x'), cmd.index('-z')
        check(abs(float(cmd[xi + 1]) - (3.0 - 0.12)) < 1e-3 and cmd[zi + 1] == '0.000',
              f'spawned under the magazine on the ground: {" ".join(cmd[-8:])}')
        mk = bus.PUBLISHED.get('/writer/beacon_markers', [])[-1]
        cols = {(round(m.color.r, 2), round(m.color.g, 2), round(m.color.b, 2)) for m in mk.markers}
        check((1.0, 0.85, 0.0) in cols and (0.1, 0.9, 0.3) in cols, 'RViz markers: gas yellow, exit green')
        # mission file
        m = load_mission(os.path.join(tmp, 'latest.json'))
        recs, rejected, _ = decode_mission(m)
        by = {r['beacon_id']: r for r in recs}
        ok_links = all((by[d['beacon_id']]['next'] or {}).get('id') == d['link_id'] for d in drops)
        check(len(recs) == len(drops) and rejected == 0 and all(len(e['frame']) == 88 for e in m['beacons']),
              f'mission file: {len(recs)} beacons, every 44-byte frame verifies')
        check(ok_links and by[gas[0]['beacon_id']]['kind'] == 'GAS' and by[0]['kind'] == 'EXIT',
              'records: next = link, kinds EXIT / WAYPOINT / GAS')
        gx, gy = by[gas[0]['beacon_id']]['x'], by[gas[0]['beacon_id']]['y']
        check(math.hypot(gx - 5.4, gy - 0.3) < 0.02, f'gas record position = the hazard ({gx:.2f}, {gy:.2f})')
    finally:
        shutil.which, subprocess.Popen = old_which, old_popen


# ====================================================================== executor
def test_executor(world_file, minutes):
    import sim2d
    import sim_mission
    import importlib
    import shutil
    import subprocess
    import tempfile
    import writer_robot.executor_node as exn
    import writer_robot.beacon_radio_sim as brs
    importlib.reload(exn)
    importlib.reload(brs)
    print(f'\n== executor_node + beacon_radio_sim driving the 2D simulator: {os.path.basename(world_file)}')
    world = sim_mission.prepare_world(world_file)
    wr = sim_mission.run_writer(world, 1, 45, True, sim2d.MIN_AREA)
    tmp = tempfile.mkdtemp()
    mfile = os.path.join(tmp, 'mission.json')
    wr['log'].save(mfile)
    n_haz = sum(1 for e in wr['log'].entries if e['record']['kind'] in ('RADIATION', 'THERMAL', 'GAS'))
    spawned = []

    class FakePopen:
        def __init__(self, cmd, **kw):
            spawned.append(cmd)
    old_which, old_popen = shutil.which, subprocess.Popen
    shutil.which = lambda name: '/usr/bin/' + name
    subprocess.Popen = FakePopen
    try:
        bus.reset()
        bus.PARAM_OVERRIDES['mission_file'] = mfile
        robot = sim_mission.Robot(world, np.random.default_rng(7))
        tf2_ros.POSE[('map', 'base_footprint')] = (0.0, 0.0, 0.0)
        brs.BeaconRadioSim()
        node = exn.ExecutorNode()
        dt, t = 0.05, 0.0
        nxt = {'scan': 0.0, 'map': 0.0, 'cam': 0.0}
        angles = robot.angles
        while t < minutes * 60:
            x, y, yaw = robot.x, robot.y, robot.yaw
            tf2_ros.POSE[('map', 'base_footprint')] = (x, y, yaw)
            if t >= nxt['scan']:
                nxt['scan'] += 1 / 8.0
                lx, ly = x + 0.10 * math.cos(yaw), y + 0.10 * math.sin(yaw)
                r = sim2d.cast(world.segs, lx, ly, angles + yaw, 12.0)
                robot.mapper.update(lx, ly, angles + yaw, r, 10.0, heading=yaw, stamp=t)
                m = LaserScan()
                m.angle_min, m.angle_max, m.angle_increment = -math.pi, math.pi, angles[1] - angles[0]
                m.range_min, m.range_max = 0.15, 12.0
                m.ranges = [float(q) if np.isfinite(q) else float('inf') for q in r]
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
            bus.run_timers(t)
            cmds = bus.PUBLISHED.get('/cmd_vel', [])
            cmd = (cmds[-1].linear.x, cmds[-1].angular.z) if cmds else (0.0, 0.0)
            robot.move(cmd, dt)
            t += dt
            if node.core.phase == 'DONE' and t - node.core.t_state > 3.0:
                break
            for k in ('/cmd_vel', '/executor/status', '/scan'):
                if len(bus.PUBLISHED.get(k, [])) > 3000:
                    bus.PUBLISHED[k] = bus.PUBLISHED[k][-5:]
        c = node.core
        print(f'     finished in {t / 60:.1f} simulated min, phase {c.phase}, '
              f'{len(c.done_targets)}/{len(c.targets)} treated')
        rx = bus.PUBLISHED.get('/executor/lora_rx', [])
        check(len({json.loads(m.data)['beacon_id'] for m in rx}) == len(wr['log'].entries),
              f'LoRa receiver: all {len(wr["log"].entries)} frames verified, decoded and delivered')
        check(robot.collisions == 0, f'no collision ({robot.collisions})')
        check(c.phase == 'DONE' and math.hypot(robot.x, robot.y) < 0.7 and len(c.done_targets) == n_haz > 0,
              f'MISSION COMPLETE: {len(c.done_targets)}/{n_haz} hazards treated, back at the exit')
        evs = bus.PUBLISHED.get('/executor/mission_events', [])
        check(len(evs) == n_haz and all('kind' in json.loads(e.data) for e in evs),
              f'/executor/mission_events: {len(evs)} treated-hazard reports')
        tr = [s_ for s_ in spawned if 'treated_marker' in ' '.join(s_)]
        check(len(tr) == n_haz, f'Gazebo: {len(tr)} green "treated" discs spawned')
        mk = bus.PUBLISHED.get('/executor/markers', [])
        ok_mk = mk and any(m.type == 5 and len(m.points) >= 2 for m in mk[-1].markers) and \
            any(m.ns == 'keepout' for m in mk[-1].markers)
        check(bool(ok_mk), '/executor/markers: beacon tree links and hazard zones')
        routes = bus.PUBLISHED.get('/executor/route', [])
        check(any(len(r_.poses) >= 3 for r_ in routes), '/executor/route: the beacons to visit')
        st = bus.PUBLISHED.get('/executor/status', [])
        check(bool(st) and 'MISSION COMPLETE' in st[-1].data, f'/executor/status ends with "{st[-1].data if st else ""}"')
    finally:
        shutil.which, subprocess.Popen = old_which, old_popen


def test_mine_sensors():
    print('\n== mine_sensors (Gafsa mine)')
    bus.reset()
    import importlib
    import writer_robot.mine_sensors_node as msn
    importlib.reload(msn)
    from nav_msgs.msg import Odometry
    node = msn.MineSensorsNode()

    def at(x, y, yaw, steps=3):
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        for cb in bus.BUS.get('/odom', []):
            cb(o)
        tf2_ros.POSE[('map', 'base_footprint')] = (x, y, yaw)
        for _ in range(steps):
            bus.run_timers(bus.CLOCK['t'] + 0.2)

    def events(kind):
        return [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/events', [])
                if json.loads(m.data)['event_type'] == kind]

    at(0.0, 0.0, 0.0, steps=6)
    air = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/air', [])]
    check(air and abs(air[-1]['o2_pct'] - 20.9) < 0.05 and air[-1]['co_ppm'] < 1.0,
          f'/writer/air at the portal: O2 {air[-1]["o2_pct"] if air else "?"} %, CO {air[-1]["co_ppm"] if air else "?"} ppm')
    # walk up the dead-end gallery: radon builds up in the stagnant air
    for y in (2.0, 3.0, 4.0, 4.6):
        at(4.4, y, math.pi / 2)
    rad = events('radiation')
    check(len(rad) == 1 and math.hypot(rad[0]['event_x'] - 4.4, rad[0]['event_y'] - 7.0) < 1.0 and
          'Bq/m3' in rad[0].get('detail', ''), f'radon pocket in the dead end found: {rad[0].get("detail") if rad else "none"}')
    # the access drift, a full turn on the spot at three places: searched, no victim
    for x_ in (1.0, 4.0, 7.0):
        for k in range(8):
            at(x_, 0.0, k * math.pi / 4, steps=1)
    srch = events('searched')
    check(any('access drift' in e['detail'] and '0 victims' in e['detail'] for e in srch),
          'victim search: "access drift: searched, 0 victims found" event')
    # a phosphate seam on the pillar face, graded by the wall probe
    at(26.4, 0.2, math.pi / 2)
    ph = [e for e in events('phosphate') if e['grade'] > 26.0]
    check(len(ph) == 1 and 27.0 <= ph[0]['grade'] <= 31.0 and 'rich' in ph[0]['detail'],
          f'rich phosphate seam graded: {ph[0]["detail"] if ph else "none"}')
    hz = [json.loads(m.data) for m in bus.PUBLISHED.get('/writer/hazard', [])]
    check(all(h['type'] != 'searched' for h in hz), 'a searched area is never a hazard sighting (nothing to avoid)')
    # a trapped miner by the east wall, seen by the thermal camera
    at(28.6, 3.6, 0.0)
    vic = events('victim')
    check(len(vic) == 1 and math.hypot(vic[0]['event_x'] - 30.75, vic[0]['event_y'] - 3.6) < 0.5,
          f'victim (body heat) placed {math.hypot(vic[0]["event_x"] - 30.75, vic[0]["event_y"] - 3.6) if vic else 99:.2f} m '
          'from where the miner is')
    # the burning loader, from the south gallery
    at(16.0, -6.8, 0.0)
    fire = events('fire')
    check(len(fire) == 1 and 'C' in fire[0]['detail'], f'fire seen by the thermal camera: '
                                                        f'{fire[0]["detail"] if fire else "none"}')
    # the same place again: one event per hazard
    at(16.3, -6.8, 0.0)
    check(len(events('fire')) == 1, 'one beacon event per hazard (no repeat)')
    # what they become on the air
    from writer_robot.mission_log import MissionLog, GeoFrame
    log = MissionLog(geo=GeoFrame(34.3159, 8.4184, 0.0), clock=lambda: 1790000000)
    b0 = {'beacon_id': 0, 'event_type': 'exit', 'x': 0.0, 'y': 0.0, 'theta': 0.0, 'link_id': None}
    log.add_drop(b0, 0.0)
    e = log.add_drop({'beacon_id': 1, 'event_type': 'phosphate', 'x': 26.4, 'y': 0.2, 'theta': 0.0, 'link_id': 0,
                      'event_x': ph[0]['event_x'], 'event_y': ph[0]['event_y'], 'grade': 29.0, 'value': 0.7}, 30.0)
    s = log.add_drop({'beacon_id': 2, 'event_type': 'searched', 'x': 5.0, 'y': 0.0, 'theta': 0.0, 'link_id': 0,
                      'event_x': 3.2, 'event_y': 0.0, 'value': 0.86, 'range_m': 0.0}, 31.0)
    check(e['record']['kind'] == 'PHOSPHATE' and e['record']['severity'] == 116,
          f'phosphate record: severity {e["record"]["severity"]} = 29.0 % P2O5 x 4')
    check(s['record']['kind'] == 'SEARCHED' and s['record']['severity'] == 86,
          f'SEARCHED record: severity {s["record"]["severity"]} = 86 % of the area seen')


def main():
    rclpy.init()
    test_vision()
    test_mine_sensors()
    test_beacons()
    test_explorer(os.path.join(ROOT, 'worlds', 'retreat_test.world'), 6, 1)
    test_explorer(os.path.join(ROOT, 'worlds', 'contaminated_zone.world'), 30, 4)
    test_executor(os.path.join(ROOT, 'worlds', 'retreat_test.world'), 5)
    test_executor(os.path.join(ROOT, 'worlds', 'contaminated_zone.world'), 15)
    ok = all(r[0] for r in RESULTS)
    print(f'\n{sum(r[0] for r in RESULTS)}/{len(RESULTS)} checks passed - {"ALL PASSED" if ok else "FAILURES"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
