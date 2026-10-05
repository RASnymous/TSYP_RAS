#!/usr/bin/env python3
"""
sim_mission.py - the whole Living Map mission in the 2D simulator (no ROS):

  1. WRITER   explores the map (explorer_core), drops beacons (beacon_logic):
              EXIT at the entrance, trail beacons, hazard beacons. Every
              beacon becomes a signed 44-byte LMB2 record (mission_log) and the
              mission file is saved.
  2. EXECUTOR enters at the exit knowing NOTHING but the 44-byte frames: it
              verifies and decodes them, builds the beacon tree, drives beacon
              by beacon to every hazard, treats it from a safe standoff, and
              returns to the exit by following the `next` hops.

    python3 tools/sim_mission.py                          # big map
    python3 tools/sim_mission.py --world worlds/retreat_test.world --png m.png
    python3 tools/sim_mission.py --mission ~/writer_robot_ws/missions/latest.json
          (Executor only, on a mission file saved by the Writer in Gazebo)
    python3 tools/sim_mission.py --mission missions/demo_big.json --wait-briefing 30 --briefing 5:25,11,20
          (v9: a Command Post briefing - these targets, in this order; T:IDS:stay to stay at the last
           one, T:abort to call the robot back; several --briefing = re-targeting during the mission)

Ends with checks (exit code 0 = all passed) and optionally a picture.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import sim2d  # noqa: E402
from writer_robot.explorer_core import ExplorerCore, Params  # noqa: E402
from writer_robot.executor_core import ExecutorCore, ExecParams  # noqa: E402
from writer_robot.beacon_logic import BeaconTrail  # noqa: E402
from writer_robot.mission_log import MissionLog, decode_mission, GeoFrame  # noqa: E402
from writer_robot.mine_site import EventFilter, RESOURCE_TYPES  # noqa: E402

EPOCH = 1790000000          # simulated unix time at the start of the Writer run
# Gazebo-like imperfections (command-line options, default off)
STRESS = {'turn_slip': 1.0,      # the robot turns this fraction of the commanded rate
          'wall_grow': 0,        # occupied map cells grown by this many 5 cm cells (noisy SLAM walls)
          'depth_scale': 1.0,    # Executor camera: measured range x this
          'bearing_bias': 0.0,   # Executor camera: bearing error (deg)
          'late': {}}            # beacon id -> time (s) its record first reaches the Executor
KIND_COLOR = {'EXIT': '#18a558', 'WAYPOINT': '#1e7ae0', 'RADIATION': '#d0d', 'THERMAL': '#e22',
              'GAS': '#b5a800', 'VICTIM': '#ec4899', 'STRUCTURAL': '#d98c1a', 'PHOSPHATE': '#8b6f47',
              'GOLD': '#f5b800', 'GEMSTONE': '#14b8a6', 'SEARCHED': '#7bd88f'}
MINE_TARGETS = 'RADIATION,THERMAL,GAS,VICTIM,STRUCTURAL'      # the Executor also props unstable roofs


def prepare_world(path):
    w = sim2d.World(path)
    w.name = path
    return prepare_world_los(w)


def prepare_world_los(w):
    w.los = {}
    for h in w.hazards:
        segs = []
        for (cx, cy, sx, sy, yw, z0, z1, name) in w.rects:
            if name != h[6]:
                segs.extend(sim2d.rect_segments(cx, cy, sx, sy, yw))
        w.los[h[6]] = np.array(segs)
    return w


class Robot:
    """Diff-drive robot + LiDAR + SLAM-like map + camera, in a 2D world."""

    def __init__(self, world, rng, start=(0.0, 0.0, 0.0), min_area=sim2d.MIN_AREA):
        self.w, self.rng = world, rng
        self.x, self.y, self.yaw = start
        self.v = self.w_ = 0.0
        self.mapper = sim2d.Mapper(world.bounds)
        self.angles = np.linspace(-math.pi, math.pi, 180)
        self.nxt = {'scan': 0.0, 'map': 0.0, 'cam': 0.0}
        self.collisions = 0
        self.traj = []
        self.min_area = min_area

    def sense(self, t, core, on_detect):
        x, y, yaw = self.x, self.y, self.yaw
        if t >= self.nxt['scan']:
            self.nxt['scan'] += 1 / 8.0
            lx, ly = x + 0.10 * math.cos(yaw), y + 0.10 * math.sin(yaw)
            r = sim2d.cast(self.w.segs, lx, ly, self.angles + yaw, 12.0)
            self.mapper.update(lx, ly, self.angles + yaw, r, 10.0, heading=yaw, stamp=t)
            noisy = r + self.rng.normal(0, 0.01, r.shape)
            core.set_scan(np.where(np.isfinite(noisy), noisy, np.inf), -math.pi,
                          self.angles[1] - self.angles[0], 0.15, 12.0, pose=(x, y, yaw))
        if t >= self.nxt['map']:
            self.nxt['map'] += 2.0
            g = self.mapper.grid()
            for _ in range(STRESS['wall_grow']):
                occ = g >= 50
                grow = occ.copy()
                grow[1:, :] |= occ[:-1, :]
                grow[:-1, :] |= occ[1:, :]
                grow[:, 1:] |= occ[:, :-1]
                grow[:, :-1] |= occ[:, 1:]
                g = np.where(grow, 100, g).astype(g.dtype)
            ki, kj = np.nonzero(g >= 0)
            i0, i1 = max(ki.min() - 20, 0), min(ki.max() + 21, g.shape[0])
            j0, j1 = max(kj.min() - 20, 0), min(kj.max() + 21, g.shape[1])
            core.set_map(g[i0:i1, j0:j1].copy(), self.mapper.res, self.mapper.ox + j0 * self.mapper.res,
                         self.mapper.oy + i0 * self.mapper.res, t)
        if t >= self.nxt['cam']:
            self.nxt['cam'] += 1 / 6.0
            for kind, ex, ey, depth in sim2d.camera_detect(self.w, x, y, yaw, self.rng, self.min_area):
                on_detect(kind, ex, ey, depth)
            for d in sim2d.mine_detect(self.w, x, y, yaw):       # mine worlds: the mine sensor suite
                on_detect(d.etype, d.x, d.y, d.range, d)
        core.set_pose(x, y, yaw)

    def move(self, cmd, dt):
        tv, tw = cmd
        tw *= STRESS['turn_slip']
        self.v += max(-2.0 * dt, min(2.0 * dt, tv - self.v))
        self.w_ += max(-3.0 * dt, min(3.0 * dt, tw - self.w_))
        nyaw = self.yaw + self.w_ * dt
        nx = self.x + self.v * math.cos(self.yaw + self.w_ * dt / 2) * dt
        ny = self.y + self.v * math.sin(self.yaw + self.w_ * dt / 2) * dt
        if self.w.collides(nx, ny, nyaw):
            self.collisions += 1
            self.v = self.w_ = 0.0
        else:
            self.x, self.y, self.yaw = nx, ny, nyaw
        self.traj.append((self.x, self.y))


# ====================================================================== writer
def run_writer(world, seed, minutes, quiet, min_area):
    rng = np.random.default_rng(seed)
    robot = Robot(world, rng, min_area=min_area)
    detect_range = math.sqrt(11290.0 / min_area)
    core = ExplorerCore(Params(camera_range=min(3.2, round(0.85 * detect_range, 2))),
                        log=(lambda m: None) if quiet else (lambda m: print(f'  [{t / 60:5.1f} min] {m}')))
    def line_free(a_, b_):
        # the Writer's SLAM map: no occupied cell on the straight line
        m = robot.mapper
        n = int(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) / 0.05) + 2
        for s_ in np.linspace(0.0, 1.0, n):
            j = int((a_[0] + (b_[0] - a_[0]) * s_ - m.ox) / m.res)
            i = int((a_[1] + (b_[1] - a_[1]) * s_ - m.oy) / m.res)
            if 0 <= i < m.H and 0 <= j < m.W and m.occupied(i, j):
                return False
        return True

    trail = BeaconTrail(line_free=line_free)
    t = 0.0
    a_ = world.mine.site['anchor'] if world.mine else None
    log = MissionLog(geo=GeoFrame(a_['lat'], a_['lon'], a_.get('map_yaw_deg', 0.0)) if a_ else None,
                     clock=lambda: EPOCH + t)
    log.world = os.path.basename(getattr(world, 'name', 'contaminated_zone.world'))
    path_m = 0.0
    marked = []                    # vision node: one beacon event per hazard (dedup 1.5 m)
    mine_filter = EventFilter()    # mine sensors node: the same, one record per 4 m of seam
    nxt_trail = 0.0
    dt = 0.05

    def on_detect(kind, ex, ey, depth, info=None):
        core.add_hazard(kind, ex, ey, t)          # (resources are ignored by the explorer)
        for b0 in trail.ensure_exit(robot.x, robot.y, robot.yaw):     # the EXIT always comes first
            log.add_drop(b0, path_m)
        if info is not None:
            if mine_filter.new_event(info):
                extra = {'grade': info.grade} if info.grade is not None else {}
                b = trail.add_event(kind, robot.x, robot.y, robot.yaw, event_x=round(ex, 3), event_y=round(ey, 3),
                                    range_m=round(info.range, 2), value=round(info.value, 3), detail=info.detail,
                                    **extra)
                if b:
                    log.add_drop(b, path_m)
                    if not quiet:
                        print(f'  [{t / 60:5.1f} min] beacon #{b["beacon_id"]} {kind}: {info.detail}')
            return
        rng_m = depth / 0.95
        if rng_m <= 4.0 and not any(k == kind and math.hypot(ex - mx, ey - my) < 1.5 for mx, my, k in marked):
            marked.append((ex, ey, kind))
            b = trail.add_event(kind, robot.x, robot.y, robot.yaw, event_x=round(ex, 3), event_y=round(ey, 3),
                                range_m=round(rng_m, 2), value=0.3)
            if b:
                log.add_drop(b, path_m)

    while t < minutes * 60:
        robot.sense(t, core, on_detect)
        if log.home is None:
            log.home = [round(robot.x, 3), round(robot.y, 3)]
        if t >= nxt_trail:
            nxt_trail += 0.2
            for b in trail.update(robot.x, robot.y, robot.yaw):
                log.add_drop(b, path_m)
        cmd = core.step(t)
        px, py = robot.x, robot.y
        robot.move(cmd, dt)
        path_m += math.hypot(robot.x - px, robot.y - py)
        t += dt
        if core.state == 'DONE' and t - core.t_state > 13.0:
            break
    return {'core': core, 'robot': robot, 'log': log, 'trail': trail, 't': t, 'path_m': path_m}


# ====================================================================== executor
def parse_briefing(spec, n):
    """'T:IDS[:stay]' or 'T:abort' -> (T, briefing dict as ona_link_node publishes it)."""
    parts = spec.split(':')
    t = float(parts[0])
    abort = len(parts) > 1 and parts[1].strip().lower() == 'abort'
    ids = [] if abort else [int(q) for q in parts[1].split(',') if q.strip()]
    stay = len(parts) > 2 and parts[2].strip().lower() == 'stay'
    return t, {'mission_id': 1000 + n, 'seq': 1, 'targets': ids, 'return_to_exit': not stay, 'abort': abort,
               'in_order': True}


def run_executor(world, mission, seed, minutes, quiet, key='demo', verbose=False, frame_error=(0.0, 0.0, 0.0),
                 radio_offset=0, briefings=(), wait_briefing=0.0):
    rng = np.random.default_rng(seed + 100)
    records, rejected, geo = decode_mission(mission, key)
    if any(frame_error):
        # the Executor's SLAM map is not exactly aligned with the Writer's:
        # it reads every lat/lon (and every `next` bearing) in a shifted,
        # rotated frame
        dx, dy, dyaw = frame_error
        geo = GeoFrame(geo.lat0 + dy / 111320.0, geo.lon0 + dx / (111320.0 * math.cos(math.radians(geo.lat0))),
                       math.degrees(geo.yaw) + dyaw)
        for r in records:
            r['x'], r['y'] = geo.to_xy(r['gps']['lat'], r['gps']['lon'])
    robot = Robot(world, rng)
    t = 0.0
    mine_kw = {'target_kinds': MINE_TARGETS, 'site': 'mine', 'priority_kinds': 'VICTIM'} if world.mine else {}
    exe = ExecutorCore(ExecParams(verbose=verbose, wait_briefing=wait_briefing, **mine_kw), geo=geo,
                       log=(lambda m: None) if quiet else (lambda m: print(f'  [{t / 60:5.1f} min] {m}')))
    dt = 0.05
    radio_i = 0
    phases = []
    min_haz = {h[6]: math.inf for h in world.targets if h[0] != 'victim'}

    def on_detect(kind, ex, ey, depth, info=None):
        if STRESS['depth_scale'] != 1.0 or STRESS['bearing_bias']:
            # a camera that measures range / bearing wrongly (robot frame)
            c, s_ = math.cos(robot.yaw), math.sin(robot.yaw)
            fx = c * (ex - robot.x) + s_ * (ey - robot.y)
            fy = -s_ * (ex - robot.x) + c * (ey - robot.y)
            d = math.hypot(fx - sim2d.CAM_X, fy) * STRESS['depth_scale']
            b = math.atan2(fy, fx - sim2d.CAM_X) + math.radians(STRESS['bearing_bias'])
            fx, fy = sim2d.CAM_X + d * math.cos(b), d * math.sin(b)
            ex, ey = robot.x + c * fx - s_ * fy, robot.y + s_ * fx + c * fy
        exe.on_sighting(kind, ex, ey, t)

    brief_i = 0
    while t < minutes * 60:
        # v9: a briefing from the Command Post (ONA -> 3 gateways -> ona_link_node)
        while brief_i < len(briefings) and t >= briefings[brief_i][0]:
            exe.set_briefing(briefings[brief_i][1], t)
            brief_i += 1
        # LoRa: the records arrive one after the other, 20 per second, over and
        # over (the receiver may join the cycle anywhere: radio_offset)
        while records and radio_i < int(t * 20) + 1:
            rec = records[(radio_i + radio_offset) % len(records)]
            if t >= STRESS['late'].get(int(rec['beacon_id']), 0.0):
                exe.add_record(rec, t)
            radio_i += 1
        robot.sense(t, exe, on_detect)
        cmd = exe.step(t)
        robot.move(cmd, dt)
        phases.append(exe.phase if exe.state != 'RETREAT' else 'RETREAT')
        for h in world.targets:
            if h[6] in min_haz:
                min_haz[h[6]] = min(min_haz[h[6]], math.hypot(h[1] - robot.x, h[2] - robot.y))
        t += dt
        if exe.phase == 'DONE' and t - exe.t_state > 2.0 and t > max(STRESS['late'].values(), default=0.0) + 3.0 \
                and brief_i >= len(briefings):
            break
    return {'exe': exe, 'robot': robot, 't': t, 'phases': phases, 'records': records,
            'rejected': rejected, 'min_haz': min_haz, 'geo': geo}


def seg_dist(px, py, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - a[0]) * ex + (py - a[1]) * ey) / L2))
    return math.hypot(a[0] + t * ex - px, a[1] + t * ey - py)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=os.path.join(ROOT, 'worlds', 'contaminated_zone.world'))
    ap.add_argument('--mission', help='use this mission file (skip the Writer run)')
    ap.add_argument('--save-mission', help='save the Writer mission file here')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--min-area', type=float, default=sim2d.MIN_AREA, help='Writer camera blob threshold (px)')
    ap.add_argument('--frame-error', default='0,0,0',
                    help='dx,dy,dyaw_deg: Executor map misaligned with the Writer map')
    ap.add_argument('--radio-offset', type=int, default=0,
                    help='the Executor starts hearing the record cycle at this record')
    ap.add_argument('--png')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--turn-slip', type=float, default=1.0, help='robots turn this fraction of the commanded rate')
    ap.add_argument('--wall-grow', type=int, default=0, help='SLAM walls this many 5 cm cells thicker (noisy map)')
    ap.add_argument('--depth-scale', type=float, default=1.0, help='Executor camera range error factor')
    ap.add_argument('--bearing-bias', type=float, default=0.0, help='Executor camera bearing error (deg)')
    ap.add_argument('--expect-skip', default='', help='ID,...: hazards that cannot be reached (they must be skipped, '
                    'the rest treated)')
    ap.add_argument('--late-records', default='', help='ID:T,...: these records reach the Executor only after T s')
    ap.add_argument('--briefing', action='append', default=[],
                    help='T:ID,ID,...[:stay] or T:abort - a Command Post briefing heard at T s (repeatable)')
    ap.add_argument('--wait-briefing', type=float, default=0.0, help='s the Executor waits for a briefing')
    ap.add_argument('--extra-box', action='append', default=[],
                    help='X,Y,SX,SY[,H]: an obstacle the Writer did not see (Executor run only; repeatable)')
    a = ap.parse_args()
    STRESS.update(turn_slip=a.turn_slip, wall_grow=a.wall_grow, depth_scale=a.depth_scale,
                  bearing_bias=a.bearing_bias,
                  late={int(k): float(v) for k, v in (q.split(':') for q in a.late_records.split(',') if q)})
    world = prepare_world(a.world)
    t0 = time.time()
    wr = None
    if a.mission:
        with open(os.path.expanduser(a.mission)) as f:
            mission = json.load(f)
        print(f'mission file {a.mission}: {len(mission["beacons"])} beacons')
    else:
        print('== WRITER: exploring and dropping beacons')
        wr = run_writer(world, a.seed, 45, a.quiet, a.min_area)
        mission = wr['log'].to_json()
        kinds = {}
        for e in mission['beacons']:
            kinds[e['record']['kind']] = kinds.get(e['record']['kind'], 0) + 1
        print(f'   {wr["t"] / 60:.1f} min, {wr["path_m"]:.0f} m, state {wr["core"].state}; '
              f'{len(mission["beacons"])} beacons: ' + ', '.join(f'{v} {k}' for k, v in sorted(kinds.items())))
        if a.save_mission:
            wr['log'].save(a.save_mission)
            print(f'   mission saved: {a.save_mission}')
    for box in a.extra_box:
        v = [float(q) for q in box.split(',')]
        world.add_box(v[0], v[1], v[2], v[3], v[4] if len(v) > 4 else 1.0)
        world = prepare_world_los(world)
        print(f'   extra obstacle for the Executor at ({v[0]:.1f}, {v[1]:.1f}), {v[2]:.1f} x {v[3]:.1f} m')
    print('== EXECUTOR: following the beacons')
    fe = tuple(float(v) for v in a.frame_error.split(','))
    briefings = [parse_briefing(b, i) for i, b in enumerate(a.briefing)]
    if briefings:
        for tb, b in briefings:
            print(f'   briefing at {tb:.0f} s: ' + ('ABORT' if b['abort'] else ' -> '.join(f'#{x}' for x in b['targets'])
                                                   + ('' if b['return_to_exit'] else ', then stay')))
    ex = run_executor(world, mission, a.seed, 45, a.quiet, verbose=a.verbose, frame_error=fe,
                      radio_offset=a.radio_offset, briefings=briefings, wait_briefing=a.wait_briefing)
    exe, rob = ex['exe'], ex['robot']

    # ---------------------------------------------------------------- checks
    tk = (MINE_TARGETS if world.mine else 'RADIATION,THERMAL,GAS,VICTIM').split(',')
    live_haz = [n for n in exe.nodes.values() if n.kind in tk]
    treated = {d['id'] for d in exe.done_targets}
    expect_skip = {int(q) for q in a.expect_skip.split(',') if q}
    live_haz = [n for n in live_haz if n.id not in expect_skip]
    # hazards of the world that the Writer reported (a record within 1.5 m)
    reported = [h for h in world.targets if any(math.hypot(n.x - h[1], n.y - h[2]) < 1.5 for n in live_haz)]
    treated_world = [h for h in reported if any(math.hypot(exe.nodes[i].x - h[1], exe.nodes[i].y - h[2]) < 1.5
                                                for i in treated)]
    edges = [(exe.nodes[n.parent].drop, n.drop) for n in exe.nodes.values()
             if n.parent is not None and n.drop and exe.nodes[n.parent].drop]
    # share of the distance driven between beacons (GOTO / RETURN) that stays
    # within 1 m of a beacon link
    near_m = total_m = 0.0
    for k in range(1, len(rob.traj)):
        if ex['phases'][k] not in ('GOTO', 'RETURN'):
            continue
        (x0, y0), (x, y) = rob.traj[k - 1], rob.traj[k]
        step = math.hypot(x - x0, y - y0)
        total_m += step
        if min((seg_dist(x, y, e0, e1) for e0, e1 in edges), default=9.0) < 1.0:
            near_m += step
    frac = near_m / total_m if total_m > 0 else 1.0
    dist_exe = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(rob.traj[:-1], rob.traj[1:]))
    print('\n================ mission report ================')
    print(f' records received : {len(ex["records"])} valid, {ex["rejected"]} rejected (HMAC)')
    print(f' beacon tree      : root = EXIT #{exe.root}, {len(edges)} links')
    print(f' executor         : {ex["t"] / 60:.1f} min, {dist_exe:.0f} m driven, final phase {exe.phase}')
    for d in exe.done_targets:
        print(f'   treated {d["kind"]:<9} #{d["id"]:<3} from {d["distance"]:.2f} m of its record'
              f'{"  (camera confirmed)" if d["confirmed_by_camera"] else ""}  at {d["t"] / 60:.1f} min')
    for bid, why in exe.skipped:
        print(f'   skipped #{bid}: {why}')
    for h in world.targets:
        if h[6] in ex['min_haz']:
            print(f'   {h[6]:<20} closest approach {ex["min_haz"][h[6]]:.2f} m')
    res_recs = {}
    for n in exe.nodes.values():
        if n.kind in ('PHOSPHATE', 'GOLD', 'GEMSTONE'):
            res_recs.setdefault(n.kind, []).append(n)
    for k, ns in sorted(res_recs.items()):
        print(f'   resource records : {k:<9} ' + ', '.join(
            f'#{n.id}' + (f' {n.rec.get("severity", 0) / 4:.1f} % P2O5' if k == 'PHOSPHATE' else '') for n in ns))
    searched = [n for n in exe.nodes.values() if n.kind == 'SEARCHED']
    if world.mine:
        print(f'   victim search    : {len(searched)}/{len(world.mine.zones)} areas searched (SEARCHED records)')
    checks = [] if not expect_skip else [
        (expect_skip <= {b for b, _ in exe.skipped}, 'unreachable hazard(s) ' + ', '.join(f'#{b}' for b in sorted(expect_skip))
         + ' skipped after trying, the mission went on')]
    if world.mine and wr:
        found = {it for _, x_, y_, it in world.resources
                 if any(math.hypot(n.x - x_, n.y - y_) < (2.5 if it.startswith('seam') else 1.0)
                        for ns in res_recs.values() for n in ns)}
        checks.append((len(found) == len(world.resources),
                       f'every resource marked with a beacon: seams and finds ({len(found)}/{len(world.resources)})'))
        checks.append((len(searched) == len(world.mine.zones),
                       f'victim search: every area searched and marked ({len(searched)}/{len(world.mine.zones)})'))
        vics = [d for d in exe.done_targets if d['kind'] == 'VICTIM']
        others = [d for d in exe.done_targets if d['kind'] != 'VICTIM']
        if vics and not briefings:
            checks.append((all(v['t'] <= o['t'] for v in vics for o in others),
                           'victims first: every victim reached before any other target'))
    checks += [
        (ex['rejected'] == 0 and len(ex['records']) > 0, 'every 44-byte frame verified (HMAC) and decoded'),
        (exe.root is not None and exe.nodes[exe.root].kind == 'EXIT', 'beacon tree rooted at the EXIT beacon')]
    end_ok = exe.phase == 'DONE' and math.hypot(rob.x, rob.y) < 0.7
    end_msg = 'MISSION COMPLETE, back at the exit'
    if not briefings:
        checks.append((len(reported) > 0 and len(treated_world) == len(reported),
                       f'every hazard the Writer reported was treated ({len(treated_world)}/{len(reported)})'))
    else:
        tb, last = briefings[-1]
        after = [d for d in exe.done_targets if d['t'] >= tb]
        before = {d['id'] for d in exe.done_targets if d['t'] < tb}
        if last['abort']:
            checks.append((all(d['t'] <= tb + 0.5 for d in after),
                           f'ABORT at {tb:.0f} s: nothing treated after it, straight back to the exit'))
        else:
            want = [x for x in last['targets'] if x not in before]
            got = [d['id'] for d in after]
            checks.append((got == want, 'briefing: targets ' + ' -> '.join(f'#{x}' for x in want)
                           + f' done in this order (got {" -> ".join(f"#{x}" for x in got) or "none"})'))
            out = [d['id'] for d in exe.done_targets if d['id'] not in last['targets'] and d['t'] >= tb]
            checks.append((not out, 'nothing outside the briefing treated after it'
                           + (f' ({", ".join(f"#{x}" for x in out)})' if out else '')))
            if not last['return_to_exit']:
                tgt = exe.nodes.get(want[-1]) if want else None
                spot = tgt.drop if tgt is not None and tgt.drop else (0.0, 0.0)
                end_ok = exe.phase == 'DONE' and math.hypot(rob.x - spot[0], rob.y - spot[1]) < 3.0
                end_msg = f'briefing complete, waiting at beacon #{want[-1] if want else "?"} as ordered'
        prog = exe.progress()
        checks.append((prog['mission_ack'] and prog['mission_id'] == last['mission_id'],
                       f'briefing {last["mission_id"]} acknowledged in the robot\'s reports '
                       f'({prog["done"]}/{prog["total"]} done)'))
    checks += [
        (rob.collisions == 0, f'no collision ({rob.collisions})'),
        (all(v >= 1.0 for v in ex['min_haz'].values()), 'never closer than 1.0 m to a hazard (victims excepted)'),
        (frac >= 0.9, f'followed the beacon chain ({100 * frac:.0f} % of the {total_m:.0f} m driven between '
                      'beacons stayed within 1 m of a beacon link)'),
        (end_ok, end_msg),
    ]
    print('\n CHECKS')
    for ok, msg in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {msg}')
    allok = all(ok for ok, _ in checks)
    print(' RESULT:', 'ALL CHECKS PASSED' if allok else 'FAIL', f'   (computed in {time.time() - t0:.0f} s)')

    if a.png:
        render(a.png, world, wr, ex, edges, allok)
        print(f' picture: {a.png}')
    return 0 if allok else 1


def render(path, world, wr, ex, edges, ok):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from matplotlib.lines import Line2D
    exe, rob = ex['exe'], ex['robot']
    panels = [('WRITER: exploring, dropping beacons', wr), ('EXECUTOR: following the beacons', ex)] if wr \
        else [('EXECUTOR: following the beacons', ex)]
    x0, x1, y0, y1 = world.bounds
    ph_ = min(9.0, max(4.5, 10.5 * (y1 - y0) / max(x1 - x0, 1.0) + 1.2))     # panel height follows the map shape
    fig, axes = plt.subplots(len(panels), 1, figsize=(12, ph_ * len(panels) + 0.6))
    axes = np.atleast_1d(axes)
    for ax, (title, run) in zip(axes, panels):
        g = run['robot'].mapper.grid()
        m = run['robot'].mapper
        ext = (m.ox, m.ox + m.W * m.res, m.oy, m.oy + m.H * m.res)
        img = np.where(g < 0, 0.8, np.where(g >= 50, 0.15, 1.0))
        ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
        for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
            col = '#666'
            for k, c in (('radiation', '#d0d'), ('fire', '#e22'), ('gas', '#eb0')):
                if name.startswith(k):
                    col = c
            if name.startswith('rubble'):
                col = '#a07850'
            ax.add_patch(Polygon(sim2d.rect_corners(cx, cy, sx, sy, yw), closed=True, color=col, alpha=0.9))
        tr = np.array(run['robot'].traj)
        if run is wr:
            ax.plot(tr[:, 0], tr[:, 1], '-', color='#9ab', lw=1.0)
        for e0, e1 in edges:
            ax.plot([e0[0], e1[0]], [e0[1], e1[1]], '-', color='#1e7ae0' if run is wr else '#8fb8ea', lw=1.2)
        for n in exe.nodes.values():
            if n.drop:
                ax.plot(n.drop[0], n.drop[1], 'o', ms=5 if n.kind == 'WAYPOINT' else 8,
                        color=KIND_COLOR.get(n.kind, '#555'), mec='k', mew=0.4)
            if n.kind == 'SEARCHED':
                # a search area: "searched, N victims" (the record's position is the area's centre)
                ax.plot(n.x, n.y, 'X', ms=13, color=KIND_COLOR['SEARCHED'], mec='#1b5e20', mew=1.0, alpha=0.9)
                continue
            if n.kind in ('PHOSPHATE', 'GOLD', 'GEMSTONE'):
                # a resource: marked, not avoided (no keep-out ring)
                ax.plot(n.x, n.y, {'PHOSPHATE': 's', 'GOLD': '*', 'GEMSTONE': 'D'}[n.kind], ms=9,
                        color=KIND_COLOR[n.kind], mec='k', mew=0.6)
                if n.drop:
                    ax.plot([n.drop[0], n.x], [n.drop[1], n.y], ':', color=KIND_COLOR[n.kind], lw=1)
                ax.annotate(f'#{n.id}', (n.x, n.y), xytext=(6, -12), textcoords='offset points', fontsize=8)
            elif n.kind not in ('WAYPOINT', 'EXIT'):
                ax.add_patch(Circle((n.x, n.y), 1.2, fill=False, ls='--', ec=KIND_COLOR.get(n.kind, 'r'), lw=1.2))
                if n.drop:
                    ax.plot([n.drop[0], n.x], [n.drop[1], n.y], ':', color=KIND_COLOR.get(n.kind, 'r'), lw=1)
                ax.annotate(f'#{n.id}', (n.x, n.y), xytext=(6, 6), textcoords='offset points', fontsize=8)
        if run is ex:
            ph = np.array(run['phases'])
            for p, c in (('GOTO', '#1e7ae0'), ('APPROACH', '#ff9500'), ('RETURN', '#18a558'), ('RETREAT', '#ff3b30')):
                sel = ph == p
                ax.scatter(tr[sel, 0], tr[sel, 1], s=1.5, c=c)
            for d in exe.done_targets:
                ax.plot(d['x'], d['y'], '*', ms=18, color='#18a558', mec='k')
            hd = [Line2D([], [], color=c, lw=3, label=l) for l, c in
                  (('to a target (beacon by beacon)', '#1e7ae0'), ('approach to standoff', '#ff9500'),
                   ('return to the exit', '#18a558'))]
            hd += [Line2D([], [], color='#8fb8ea', lw=1.5, label='beacon links ("next")'),
                   Line2D([], [], marker='*', ls='', ms=12, color='#18a558', mec='k', label='hazard treated'),
                   Line2D([], [], marker='o', ls='', color=KIND_COLOR['EXIT'], mec='k', label='EXIT beacon')]
            ax.legend(handles=hd, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=9)
        else:
            hd = [Line2D([], [], color='#9ab', lw=2, label='Writer path'),
                  Line2D([], [], color='#1e7ae0', lw=1.5, label='beacon links ("next")')]
            hd += [Line2D([], [], marker='o', ls='', color=c, mec='k', label=k.lower())
                   for k, c in KIND_COLOR.items() if k != 'VICTIM']
            ax.legend(handles=hd, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=9)
        sim2d.draw_mine(ax, world)
        ax.plot(0, 0, 'ks', ms=8, mfc='none')
        ax.set_aspect('equal')
        ax.set_title(title + (f' - {run["t"] / 60:.1f} min' if run else ''))
    fig.suptitle(f'Living Map mission - {os.path.basename(world_path_name(world))} - {"PASS" if ok else "FAIL"}')
    fig.tight_layout()
    fig.savefig(path, dpi=100)


def world_path_name(world):
    return getattr(world, 'name', 'world')


if __name__ == '__main__':
    sys.exit(main())
