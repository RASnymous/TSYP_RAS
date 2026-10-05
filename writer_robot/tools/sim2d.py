#!/usr/bin/env python3
"""
sim2d.py - 2D test simulator for the Writer robot's explorer (no ROS, no Gazebo).

Runs the REAL exploration brain (writer_robot/explorer_core.py) against a 2D
model of the Gazebo world: the same SDF file, the same robot footprint, a
360-degree LiDAR (180 rays, 12 m, including the robot's own magazine hitting
the rear rays), a SLAM-like occupancy grid published every 2 s, the RGB-D
camera's hazard detection (field of view, line of sight, minimum blob size),
and the diff-drive's acceleration limits.

    python3 tools/sim2d.py                                   # the big world
    python3 tools/sim2d.py --world worlds/contaminated_zone_small.world
    python3 tools/sim2d.py --png run.png --seed 3 --minutes 40

It prints the explorer's decisions as they happen and ends with checks:
no collision, never closer than 0.9 m to a hazard centre after spotting it,
retreat on every hazard in the way, exploration complete, back at the start.
Exit code 0 = all checks passed. Needs numpy (matplotlib only for --png).
"""
import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from writer_robot.explorer_core import ExplorerCore, Params  # noqa: E402
from writer_robot.mine_site import MineSite, site_file_for, RESOURCE_TYPES  # noqa: E402

# robot geometry (from urdf/writer_robot.urdf.xacro)
BODY_X = (-0.20, 0.226)       # chassis 0.40 long + camera housing at the front
BODY_Y = (-0.19, 0.19)        # wheels at +-0.17, 0.04 wide
LIDAR_X, LIDAR_Z = 0.10, 0.34    # LiDAR on its mast, above the magazine and antenna
CAM_X, CAM_Z = 0.213, 0.135
HFOV, IMG_W, IMG_H = 1.089, 320, 240
VFOV = 2 * math.atan(math.tan(HFOV / 2) * IMG_H / IMG_W)
MIN_AREA = 800        # vision node min_area_px
KARTO_MIN_PASS = 2    # slam_toolbox min_pass_through (its default)
MAGAZINE = (-0.17, -0.07, -0.05, 0.05)   # x0, x1, y0, y1 in base frame (--self-hits test)


class World:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        world = root.find('world')
        self.rects = []      # (cx, cy, sx, sy, yaw, zmin, zmax, name)
        self.hazards = []    # (kind, x, y, sx, sy, sz, name)
        for m in world.findall('model'):
            name = m.get('name')
            if m.findtext('static', 'false').strip() != 'true' or name == 'ground_plane':
                continue
            pose = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
            for col in m.iter('collision'):
                box = col.find('geometry/box/size')
                if box is None:
                    continue
                sx, sy, sz = [float(v) for v in box.text.split()]
                cp = [float(v) for v in (col.findtext('pose') or '0 0 0 0 0 0').split()]
                yaw = pose[5] + cp[5]
                cx = pose[0] + cp[0] * math.cos(pose[5]) - cp[1] * math.sin(pose[5])
                cy = pose[1] + cp[0] * math.sin(pose[5]) + cp[1] * math.cos(pose[5])
                cz = pose[2] + cp[2]
                self.rects.append((cx, cy, sx, sy, yaw, cz - sz / 2, cz + sz / 2, name))
                for kind in ('radiation', 'fire', 'gas'):
                    if name.startswith(kind):
                        self.hazards.append((kind, cx, cy, sx, sy, sz, name))
        self._finish()
        # a mine world (its site file next to it): hazards and resources measured by the mine sensors
        site = site_file_for(path)
        self.mine = MineSite(site, path) if site else None
        self.targets = list(self.hazards) + (
            [(k, x, y, 0.4, 0.4, 0.8, iid) for k, x, y, iid in self.mine.hazards()] if self.mine else [])
        self.resources = [(k, x, y, iid) for k, x, y, iid in self.mine.resources()] if self.mine else []

    def add_box(self, cx, cy, sx, sy, height=1.0, name='extra_box'):
        """An obstacle that is not in the SDF (e.g. debris that fell after the Writer's run)."""
        self.rects.append((cx, cy, sx, sy, 0.0, 0.0, height, name))
        self._finish()

    def _finish(self):
        segs = []
        for (cx, cy, sx, sy, yaw, z0, z1, name) in self.rects:
            if not (z0 <= LIDAR_Z <= z1):
                continue
            segs.extend(rect_segments(cx, cy, sx, sy, yaw))
        self.segs = np.array(segs)     # (N, 4): x1 y1 x2 y2
        xs = [s[0] for s in segs] + [s[2] for s in segs]
        ys = [s[1] for s in segs] + [s[3] for s in segs]
        self.bounds = (min(xs) - 1.0, max(xs) + 1.0, min(ys) - 1.0, max(ys) + 1.0)
        # obstacles the robot body can touch (everything static reaching below 0.30 m)
        solid = [r for r in self.rects if r[5] < 0.30]
        self.polys = [rect_corners(cx, cy, sx, sy, yaw) for (cx, cy, sx, sy, yaw, _, _, _) in solid]
        self.rc = np.array([(cx, cy, sx / 2, sy / 2, math.cos(yaw), math.sin(yaw)) for (cx, cy, sx, sy, yaw, _, _, _) in solid])

    def collides(self, x, y, yaw):
        """Robot body (0.40 x 0.38 incl. wheels) overlaps a static box?"""
        rc = self.rc
        dx, dy = x - rc[:, 0], y - rc[:, 1]
        lx = np.abs(dx * rc[:, 4] + dy * rc[:, 5])
        ly = np.abs(-dx * rc[:, 5] + dy * rc[:, 4])
        near = np.nonzero((lx < rc[:, 2] + 0.3) & (ly < rc[:, 3] + 0.3))[0]
        if near.size == 0:
            return False
        body = rect_corners(x + (BODY_X[0] + BODY_X[1]) / 2, y, BODY_X[1] - BODY_X[0], BODY_Y[1] - BODY_Y[0], yaw)
        return any(polys_overlap(body, self.polys[k]) for k in near)


def rect_corners(cx, cy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
        pts.append((cx + c * dx - s * dy, cy + s * dx + c * dy))
    return np.array(pts)


def rect_segments(cx, cy, sx, sy, yaw):
    p = rect_corners(cx, cy, sx, sy, yaw)
    return [(p[i][0], p[i][1], p[(i + 1) % 4][0], p[(i + 1) % 4][1]) for i in range(4)]


def polys_overlap(a, b):
    """Separating-axis test for two convex polygons (4x2 arrays)."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def cast(segs, ox, oy, angles, rmax):
    """Ray casting: distance to the first segment for each angle (inf = none)."""
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    x1, y1, x2, y2 = segs[:, 0][None, :], segs[:, 1][None, :], segs[:, 2][None, :], segs[:, 3][None, :]
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf)
    d = t.min(axis=1)
    d[d > rmax] = np.inf
    return d


class SimpleMapper:
    """Optimistic SLAM stand-in: log-odds grid from EVERY scan and every ray (--mapper simple)."""

    def __init__(self, bounds, res=0.05):
        x0, x1, y0, y1 = bounds
        self.res, self.ox, self.oy = res, x0, y0
        self.W = int(math.ceil((x1 - x0) / res))
        self.H = int(math.ceil((y1 - y0) / res))
        self.L = np.zeros((self.H, self.W), dtype=np.float32)
        self.seen = np.zeros((self.H, self.W), dtype=bool)

    def update(self, ox, oy, angles, ranges, rmax):
        """Free along every ray, occupied at the hit (vectorised over all rays)."""
        step = self.res * 0.8
        hit = np.isfinite(ranges)
        rr = np.where(hit, np.minimum(ranges, rmax), rmax)
        n = int(rmax / step) + 1
        ts = np.arange(n) * step
        ca, sa = np.cos(angles), np.sin(angles)
        use = ts[None, :] < (rr - 0.05)[:, None]
        xs = ox + ts[None, :] * ca[:, None]
        ys = oy + ts[None, :] * sa[:, None]
        j = ((xs[use] - self.ox) / self.res).astype(int)
        i = ((ys[use] - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        self.L[i[ok], j[ok]] -= 0.4
        self.seen[i[ok], j[ok]] = True
        hx = ox + ranges[hit] * ca[hit]
        hy = oy + ranges[hit] * sa[hit]
        j = ((hx - self.ox) / self.res).astype(int)
        i = ((hy - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        self.L[i[ok], j[ok]] += 1.2
        self.seen[i[ok], j[ok]] = True
        np.clip(self.L, -4.0, 4.0, out=self.L)

    def occupied(self, i, j):
        return self.L[i, j] > 0.6

    def grid(self):
        g = np.full((self.H, self.W), -1, dtype=np.int8)
        g[self.seen & (self.L < 0.0)] = 0
        g[self.L > 0.6] = 100
        return g


class KartoMapper:
    """slam_toolbox (Karto) stand-in. What matters for the explorer:
    * a scan is added to the map only after the robot moved >= min_travel m or
      turned >= min_heading rad since the last added scan (the first scan is
      always added), and at most one every min_interval s (slam_toolbox's
      minimum_time_interval): a robot standing still does NOT improve its map;
    * each ray counts a PASS in every cell it crosses (the hit cell too) and a
      HIT in its end cell; a cell stays UNKNOWN until more than `min_pass`
      rays have crossed it, then it is occupied if hits / passes > 0.1, else
      free. With 180 beams (2 deg apart) a single scan crosses a 5 cm cell more
      than twice only within ~0.6 m of the LiDAR, so the first map is a small
      disc: far walls appear only after the robot has turned or moved."""

    def __init__(self, bounds, res=0.05, min_pass=2, min_travel=0.3, min_heading=0.3, min_interval=0.5):
        x0, x1, y0, y1 = bounds
        self.res, self.ox, self.oy = res, x0, y0
        self.W = int(math.ceil((x1 - x0) / res))
        self.H = int(math.ceil((y1 - y0) / res))
        self.passes = np.zeros(self.H * self.W, dtype=np.int32)
        self.hits = np.zeros(self.H * self.W, dtype=np.int32)
        self.min_pass, self.min_travel, self.min_heading = min_pass, min_travel, min_heading
        self.min_interval = min_interval      # slam_toolbox minimum_time_interval (s)
        self.last = None
        self.last_t = None
        self.added = 0

    def update(self, ox, oy, angles, ranges, rmax, heading=None, stamp=None):
        if heading is None:
            heading = float(angles[len(angles) // 2])
        if self.last is not None:
            lx, ly, lh = self.last
            dh = abs(math.atan2(math.sin(heading - lh), math.cos(heading - lh)))
            if math.hypot(ox - lx, oy - ly) < self.min_travel and dh < self.min_heading:
                return
            if stamp is not None and self.last_t is not None and stamp - self.last_t < self.min_interval:
                return
        self.last = (ox, oy, heading)
        self.last_t = stamp
        self.added += 1
        hit = np.isfinite(ranges) & (ranges < rmax)
        rr = np.where(hit, ranges, rmax)
        step = self.res * 0.5
        n = int(rmax / step) + 2
        ts = np.arange(n) * step
        ca, sa = np.cos(angles), np.sin(angles)
        use = ts[None, :] <= rr[:, None]
        xs = ox + ts[None, :] * ca[:, None]
        ys = oy + ts[None, :] * sa[:, None]
        j = np.floor((xs - self.ox) / self.res).astype(np.int64)
        i = np.floor((ys - self.oy) / self.res).astype(np.int64)
        ok = use & (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        ray = np.broadcast_to(np.arange(len(angles))[:, None], ok.shape)
        cell = i * self.W + j
        key = np.unique(ray[ok].astype(np.int64) * (self.H * self.W) + cell[ok])
        np.add.at(self.passes, key % (self.H * self.W), 1)
        hx, hy = ox + rr[hit] * ca[hit], oy + rr[hit] * sa[hit]
        hj = np.floor((hx - self.ox) / self.res).astype(np.int64)
        hi = np.floor((hy - self.oy) / self.res).astype(np.int64)
        hok = (hi >= 0) & (hi < self.H) & (hj >= 0) & (hj < self.W)
        hc = hi[hok] * self.W + hj[hok]
        np.add.at(self.hits, hc, 1)
        # the end cell always counts as passed (the sampling may just miss it)
        endp = np.unique(hc)
        miss = endp[self.passes[endp] < self.hits[endp]]
        self.passes[miss] = self.hits[miss]

    def occupied(self, i, j):
        k = i * self.W + j
        return self.passes[k] > self.min_pass and self.hits[k] > 0.1 * self.passes[k]

    def grid(self):
        g = np.full(self.H * self.W, -1, dtype=np.int8)
        known = self.passes > self.min_pass
        ratio = np.where(self.passes > 0, self.hits / np.maximum(self.passes, 1), 0.0)
        g[known & (ratio <= 0.1)] = 0
        g[known & (ratio > 0.1)] = 100
        return g.reshape(self.H, self.W)


# the other simulators and the node tests use this one
Mapper = KartoMapper


def camera_detect(world, x, y, yaw, rng, min_area=MIN_AREA):
    """Hazards the RGB-D camera + vision node would report: list of
    (kind, est_x, est_y, depth) in the map frame, computed like the node."""
    cx, cy = x + CAM_X * math.cos(yaw), y + CAM_X * math.sin(yaw)
    out = []
    for kind, hx, hy, sx, sy, sz, name in world.hazards:
        d = math.hypot(hx - cx, hy - cy)
        b = math.atan2(hy - cy, hx - cx) - yaw
        b = math.atan2(math.sin(b), math.cos(b))
        if abs(b) > HFOV / 2 or d > 8.0:
            continue
        face = max(0.15, d - sx / 2)
        wpx = 2 * math.atan(sx / 2 / face) / HFOV * IMG_W
        up = min(math.atan((sz - CAM_Z) / face), VFOV / 2)
        down = min(math.atan(CAM_Z / face), VFOV / 2)
        hpx = (up + down) / VFOV * IMG_H
        if wpx * hpx * 0.9 < min_area:
            continue
        # line of sight to the face centre, ignoring the hazard itself
        tx, ty = hx - (sx / 2) * math.cos(b + yaw), hy - (sx / 2) * math.sin(b + yaw)
        segs = world.los[name]
        a = math.atan2(ty - cy, tx - cx)
        dist = cast(segs, cx, cy, np.array([a]), 20.0)[0]
        if dist < math.hypot(tx - cx, ty - cy) - 0.05:
            continue
        depth = face * math.cos(b) * (1.0 + rng.normal(0, 0.03))
        # vision node: forward = depth, lateral = depth * tan(bearing), in base frame + camera offset
        fx = CAM_X + depth
        fy = depth * math.tan(b)
        ex = x + fx * math.cos(yaw) - fy * math.sin(yaw)
        ey = y + fx * math.sin(yaw) + fy * math.cos(yaw)
        out.append((kind, ex, ey, depth))
    return out


def mine_detect(world, x, y, yaw):
    """The mine sensor suite (mine worlds only): list of mine_site.Detection."""
    return world.mine.detect(x, y, yaw) if world.mine is not None else []


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=os.path.join(ROOT, 'worlds', 'contaminated_zone.world'))
    ap.add_argument('--minutes', type=float, default=45.0, help='simulated time limit')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--start', default='0,0,0', help='x,y,yaw of the robot')
    ap.add_argument('--mapper', default='karto', choices=('karto', 'simple'),
                    help='karto: slam_toolbox-like map (scans added only after moving/turning, cells known '
                         'after min-pass rays); simple: every scan, every ray')
    ap.add_argument('--no-turn-scans', action='store_true',
                    help='karto mapper: never add scans when only turning (what happens when the odometry '
                         'does not see the robot turn)')
    ap.add_argument('--min-pass', type=int, default=KARTO_MIN_PASS,
                    help='karto mapper: a cell is known after MORE than this many rays crossed it')
    ap.add_argument('--png', help='save a picture of the run')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--min-area', type=float, default=MIN_AREA,
                    help='vision blob threshold in pixels (bigger = hazards seen later)')
    ap.add_argument('--expect-turn-back', action='store_true',
                    help='fail unless the robot drove away (>= 1 m) within 30 s of seeing each hazard')
    ap.add_argument('--verbose', action='store_true', help='log every planning decision')
    ap.add_argument('--trace', type=float, default=0.0, help='print pose/command every N seconds')
    ap.add_argument('--self-hits', action='store_true', help='add LiDAR hits on the robot\'s own magazine')
    ap.add_argument('--turn-slip', type=float, default=1.0,
                    help='the robot turns this fraction of the commanded rate (skid-steer wheels slide in turns)')
    ap.add_argument('--ctl-hz', type=float, default=10.0,
                    help='control loop rate (a busy computer runs the explorer node less often)')
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    world = World(a.world)
    # camera line of sight: every box except the hazard being looked at
    world.los = {}
    for h in world.hazards:
        segs = []
        for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
            if name != h[6]:
                segs.extend(rect_segments(cx, cy, sx, sy, yw))
        world.los[h[6]] = np.array(segs)

    mapper = SimpleMapper(world.bounds) if a.mapper == 'simple' else \
        KartoMapper(world.bounds, min_pass=a.min_pass, min_heading=1e9 if a.no_turn_scans else 0.3)
    logs = []

    def log(msg):
        logs.append((t, msg))
        if not a.quiet:
            print(f'[{t / 60:5.1f} min] {msg}')

    # the explorer's camera coverage range must match what the vision node
    # really detects: a 0.3 x 0.6 m block covers ~11300/d^2 pixels at d metres
    detect_range = math.sqrt(11290.0 / a.min_area)
    params = Params(verbose=a.verbose, camera_range=min(3.2, round(0.85 * detect_range, 2)))
    core = ExplorerCore(params, log=log)
    x, y, yaw = [float(v) for v in a.start.split(',')]
    v = w = 0.0
    dt = 0.05
    t = 0.0
    angles = np.linspace(-math.pi, math.pi, 180)     # gz: 180 samples over [-pi, pi]
    ainc = angles[1] - angles[0]
    traj, states = [], []
    collisions = 0
    first_seen = {}
    min_after_seen = {}
    min_ever = {h[6]: math.inf for h in world.targets}
    res_seen = {}
    next_map, next_scan, next_cam, next_ctl = 0.0, 0.0, 0.0, 0.0
    cmd = (0.0, 0.0)
    t_wall = time.time()
    plan_times = []
    still = longest_still = 0.0
    away = {}

    while t < a.minutes * 60.0:
        # sensors
        if t >= next_scan:
            next_scan += 1 / 8.0
            lx, ly = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
            ranges = cast(world.segs, lx, ly, angles + yaw, 12.0)
            if a.mapper == 'simple':
                mapper.update(lx, ly, angles + yaw, ranges, 10.0)
            else:
                mapper.update(lx, ly, angles + yaw, ranges, 10.0, heading=yaw, stamp=t)
            scan = ranges
            if a.self_hits:
                # old LiDAR mounting: the beacon magazine cuts the scan plane behind
                # the LiDAR. The explorer must ignore these points.
                mc = (MAGAZINE[0] + MAGAZINE[1]) / 2
                mag = rect_corners(x + mc * math.cos(yaw), y + mc * math.sin(yaw),
                                   MAGAZINE[1] - MAGAZINE[0], MAGAZINE[3] - MAGAZINE[2], yaw)
                mseg = np.array([(mag[i][0], mag[i][1], mag[(i + 1) % 4][0], mag[(i + 1) % 4][1]) for i in range(4)])
                scan = np.minimum(scan, cast(mseg, lx, ly, angles + yaw, 12.0))
            noisy = scan + rng.normal(0, 0.01, scan.shape)
            core.set_scan(np.where(np.isfinite(noisy), noisy, np.inf), -math.pi, ainc, 0.15, 12.0,
                          pose=(x, y, yaw))
        if t >= next_map:
            next_map += 2.0
            g = mapper.grid()
            # like slam_toolbox: the published map only covers the known area
            # (it grows, and its origin moves, as the robot explores)
            ki, kj = np.nonzero(g >= 0)
            m = 20
            i0, i1 = max(ki.min() - m, 0), min(ki.max() + m + 1, g.shape[0])
            j0, j1 = max(kj.min() - m, 0), min(kj.max() + m + 1, g.shape[1])
            core.set_map(g[i0:i1, j0:j1].copy(), mapper.res, mapper.ox + j0 * mapper.res,
                         mapper.oy + i0 * mapper.res, t)
        if t >= next_cam:
            next_cam += 1 / 6.0
            dets = camera_detect(world, x, y, yaw, rng, a.min_area)
            for d in mine_detect(world, x, y, yaw):
                if d.etype == 'searched':
                    if 'search:' + d.item not in res_seen:
                        res_seen['search:' + d.item] = (t, d.detail)
                        log(f'area searched: {d.detail}')
                    continue
                if d.etype in RESOURCE_TYPES:
                    if d.item not in res_seen:
                        res_seen[d.item] = (t, d.detail)
                        log(f'resource marked: {d.etype} ({d.detail})')
                else:
                    dets.append(d.as_tuple())
            for kind, ex, ey, depth in dets:
                core.add_hazard(kind, ex, ey, t)
                for h in world.targets:
                    if h[0] == kind and math.hypot(h[1] - ex, h[2] - ey) < 1.5 and h[6] not in first_seen:
                        first_seen[h[6]] = (t, math.hypot(h[1] - x, h[2] - y))
        core.set_pose(x, y, yaw)
        if t >= next_ctl:
            next_ctl += 1.0 / a.ctl_hz
            t0 = time.time()
            cmd = core.step(t)
            el = time.time() - t0
            if el > 0.02:
                plan_times.append(el)
        # dynamics with the diff drive's acceleration limits
        tv, tw = cmd
        tw *= a.turn_slip
        v += max(-2.0 * dt, min(2.0 * dt, tv - v))
        w += max(-3.0 * dt, min(3.0 * dt, tw - w))
        nyaw = yaw + w * dt
        nx = x + v * math.cos(yaw + w * dt / 2) * dt
        ny = y + v * math.sin(yaw + w * dt / 2) * dt
        if world.collides(nx, ny, nyaw):
            collisions += 1
            v = w = 0.0
            if collisions <= 5:
                log(f'COLLISION at ({nx:.2f}, {ny:.2f})')
        else:
            x, y, yaw = nx, ny, nyaw
        for h in world.targets:
            d = math.hypot(h[1] - x, h[2] - y)
            min_ever[h[6]] = min(min_ever[h[6]], d)
            if h[6] in first_seen:
                min_after_seen[h[6]] = min(min_after_seen.get(h[6], math.inf), d)
                if t - first_seen[h[6]][0] <= 30.0:
                    away[h[6]] = max(away.get(h[6], 0.0), d - first_seen[h[6]][1])
        traj.append((x, y))
        states.append(core.state)
        if core.state != 'DONE' and abs(v) < 0.02 and abs(w) < 0.05:
            still += dt
            longest_still = max(longest_still, still)
        else:
            still = 0.0
        if a.trace and int(t / a.trace) != int((t - dt) / a.trace):
            print(f'  t={t:6.1f} pose=({x:5.2f},{y:5.2f},{math.degrees(yaw):6.1f}) cmd=({cmd[0]:.2f},{cmd[1]:.2f}) '
                  f'v={v:.2f} {core.state:8s} {core.status}')
        t += dt
        if core.state == 'DONE' and core.done_checked:
            break
        if core.state == 'DONE' and t - core.t_state > 14.0:
            break

    # ------------------------------------------------------------ report
    wall = time.time() - t_wall
    retreats = [e for e in core.events if e[1].startswith('RETREAT')]
    complete = any('EXPLORATION COMPLETE' in e[1] for e in core.events)
    home = core.home or (0, 0)
    at_home = math.hypot(x - home[0], y - home[1]) < 0.6
    g = mapper.grid()
    known = (g >= 0).sum() * mapper.res ** 2
    path_len = sum(math.hypot(traj[i + 1][0] - traj[i][0], traj[i + 1][1] - traj[i][1]) for i in range(0, len(traj) - 1, 1))
    print('\n================ sim2d report ================')
    print(f' world            : {os.path.basename(a.world)}  ({len(world.rects)} boxes, {len(world.targets)} hazards'
          + (f', {len(world.resources)} resources' if world.mine else '') + ')')
    print(f' simulated time   : {t / 60:.1f} min   (computed in {wall:.0f} s)')
    print(f' distance driven  : {path_len:.0f} m')
    print(f' map known area   : {known:.0f} m2')
    print(f' plans            : {core.plans}  (slowest planning step {max(plan_times, default=0) * 1000:.0f} ms)')
    print(f' retreats         : {len(retreats)}')
    for h in world.targets:
        fs = first_seen.get(h[6])
        print(f'   {h[6]:<20} first seen {"never" if fs is None else f"at {fs[0] / 60:.1f} min from {fs[1]:.1f} m"}'
              f'   closest after seeing it {min_after_seen.get(h[6], math.inf):.2f} m   closest ever {min_ever[h[6]]:.2f} m')
    st = np.array(states)
    print(' time per state   : ' + ', '.join(f'{k.lower()} {(st == k).sum() * dt / 60:.1f} min'
                                          for k in ('FOLLOW', 'LOOK', 'PLAN', 'RETREAT', 'RECOVER', 'HOME', 'DONE')
                                          if (st == k).any()))
    print(f' final state      : {core.state} - {core.status}')
    checks = []
    checks.append((collisions == 0, f'no collision ({collisions} blocked moves)'))
    for k, x_, y_, iid in world.resources:
        rs = res_seen.get(iid)
        print(f'   {iid:<20} {"marked at " + format(rs[0] / 60, ".1f") + " min: " + rs[1] if rs else "NOT found"}')
    if world.mine is not None and world.mine.zones:
        for zid, name, frac, nv in world.mine.coverage():
            rs = res_seen.get('search:' + zid)
            print(f'   {zid} {name:<24} {100 * frac:4.0f} % seen' + (f', searched at {rs[0] / 60:.1f} min' if rs else '')
                  + (f', {nv} victim(s)' if nv else ''))
    ok_dist = all(min_after_seen.get(h[6], math.inf) >= 0.9 for h in world.targets)
    checks.append((ok_dist, 'never within 0.9 m of a hazard centre after spotting it'))
    checks.append((len(first_seen) == len(world.targets), f'every hazard spotted ({len(first_seen)}/{len(world.targets)})'))
    if world.mine:
        n_search = sum(1 for k in res_seen if k.startswith('search:'))
        checks.append((n_search == len(world.mine.zones),
                       f'victim search: every area searched ({n_search}/{len(world.mine.zones)})'))
        checks.append((len(res_seen) - n_search == len(world.resources),
                       f'every resource found: phosphate seams and finds ({len(res_seen) - n_search}/{len(world.resources)})'))
    if a.expect_turn_back:
        ok_tb = all(away.get(h[6], 0.0) >= 1.0 for h in world.hazards)
        checks.append((ok_tb, 'turned back after seeing each hazard (>= 1 m away within 30 s): ' +
                       ', '.join(f'{h[6]} +{away.get(h[6], 0.0):.1f} m' for h in world.hazards)))
    checks.append((longest_still < 20.0, f'never stood still for long (longest {longest_still:.1f} s)'))
    checks.append((complete, 'exploration completed (no reachable frontier left)'))
    checks.append((at_home and core.state == 'DONE', 'back at the start and stopped'))
    print('\n CHECKS')
    for ok, msg in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {msg}')
    allok = all(ok for ok, _ in checks)
    print(' RESULT:', 'ALL CHECKS PASSED' if allok else 'FAIL')

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
        fig, ax = plt.subplots(figsize=(13, 10))
        ext = (mapper.ox, mapper.ox + mapper.W * mapper.res, mapper.oy, mapper.oy + mapper.H * mapper.res)
        img = np.where(g < 0, 0.75, np.where(g >= 50, 0.1, 1.0))
        ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
        for (cx, cy, sx, sy, yw, z0, z1, name) in world.rects:
            col = '#555'
            for k, c in (('radiation', '#d0d'), ('fire', '#e22'), ('gas', '#eb0')):
                if name.startswith(k):
                    col = c
            if name.startswith('rubble'):
                col = '#a07850'
            ax.add_patch(Polygon(rect_corners(cx, cy, sx, sy, yw), closed=True, color=col, alpha=0.9))
        for h in core.hazards:
            ax.add_patch(Circle((h.x, h.y), core.p.keepout_radius, fill=False, ls='--', ec='#f55', lw=1.5))
        draw_mine(ax, world)
        tr = np.array(traj)
        st = np.array(states)
        for s, c in (('FOLLOW', '#1e7ae0'), ('HOME', '#18a558'), ('RETREAT', '#ff3b30'), ('RECOVER', '#ff9500')):
            m = st == s
            ax.scatter(tr[m, 0], tr[m, 1], s=1.2, c=c, label=s.lower())
        ax.plot(tr[0, 0], tr[0, 1], 'o', ms=10, mfc='none', mec='k')
        ax.plot(tr[-1, 0], tr[-1, 1], 'kx', ms=10)
        ax.set_aspect('equal')
        ax.set_title(f'Writer robot explorer - {os.path.basename(a.world)} - {t / 60:.1f} min, '
                     f'{path_len:.0f} m, {len(retreats)} retreats - {"PASS" if allok else "FAIL"}')
        from matplotlib.lines import Line2D
        hd = [Line2D([], [], color=c, lw=3, label=l) for l, c in
              (('exploring', '#1e7ae0'), ('returning home', '#18a558'), ('retreat (way it came)', '#ff3b30'),
               ('recovery', '#ff9500'))]
        hd += [Line2D([], [], ls='--', color='#f55', label='hazard keep-out'),
               Line2D([], [], marker='o', ls='', mfc='none', mec='k', label='start'),
               Line2D([], [], marker='x', ls='', color='k', label='end')]
        ax.legend(handles=hd, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=9)
        fig.tight_layout()
        fig.savefig(a.png, dpi=110)
        print(f' picture          : {a.png}')
    return 0 if allok else 1


MINE_COLORS = {'radiation': '#c026d3', 'gas': '#9acd32', 'fire': '#e22', 'structural': '#d98c1a', 'victim': '#ec4899',
               'phosphate': '#8b6f47', 'gold': '#f5b800', 'gemstone': '#14b8a6'}


def draw_mine(ax, world):
    """Mine worlds: the invisible hazards and the resources of the site file."""
    if world.mine is None:
        return
    for it in world.mine.items:
        t = it['type']
        if t == 'phosphate':
            ax.plot([it['x0'], it['x1']], [it['y0'], it['y1']], '-', lw=5, color=MINE_COLORS['phosphate'], alpha=0.9)
            ax.annotate(f'{it["grade_p2o5"]:.0f} % P2O5', ((it['x0'] + it['x1']) / 2, it['y0']), fontsize=7,
                        xytext=(0, 6 if it['y0'] < 0 else -11), textcoords='offset points', ha='center')
            continue
        et = {'radon': 'radiation', 'roof': 'structural'}.get(t, t)
        mk = {'gold': '*', 'gemstone': 'D', 'victim': 'P'}.get(t, 'o')
        ax.plot(it['x'], it['y'], mk, ms=11 if mk != 'o' else 7, color=MINE_COLORS[et], mec='k', mew=0.5)
        if t in ('gas', 'radon'):
            from matplotlib.patches import Circle
            ax.add_patch(Circle((it['x'], it['y']), 2.5, color=MINE_COLORS[et], alpha=0.12))
        if t not in ('gold', 'gemstone'):
            ax.annotate(t, (it['x'], it['y']), xytext=(5, -10), textcoords='offset points', fontsize=7)


if __name__ == '__main__':
    sys.exit(main())
