"""
explorer_core.py - the Writer robot's exploration brain (no ROS inside).

The ROS node (frontier_explorer.py) feeds it the SLAM map, the robot pose,
the LiDAR scan and hazard sightings, and sends its (v, w) command to /cmd_vel.
The same class runs in the 2D test simulator (tools/sim2d.py), so the
behaviour is tested exactly as it runs on the robot.

WHAT IT DOES
  EXPLORE   Frontier exploration on the SLAM occupancy grid: find the edges
            between known-free and unknown space, pick the cheapest reachable
            one (Dijkstra over an inflated, cost-shaped grid) and drive there
            with a pure-pursuit path follower. Repeat.
  HAZARDS   Every hazard block the camera reports becomes a KEEP-OUT disc
            (default radius 1.2 m) that paths and goals may not enter.
  RETREAT   When a new hazard is in the robot's way, the robot RETURNS THE WAY
            IT CAME: it backs off a little, turns round and drives back along
            its own recorded trajectory until it is well clear, then goes on
            exploring the rest of the map (other routes, other rooms).
  RECOVER   If it stops making progress it backs up / turns and replans; a
            goal that fails twice is blacklisted.
  FINISHED  When no reachable frontier is left, the map is complete: it drives
            back to where it started and stops ("EXPLORATION COMPLETE").

Safety net: a LiDAR check stops forward motion when something is closer than
`emergency_dist` in front, whatever the planner says. LiDAR points inside the
robot's own footprint (the beacon magazine and antenna are above the LiDAR
plane) are ignored.
"""
import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

SQ2 = math.sqrt(2.0)
INF = float('inf')
RESOURCE_EVENTS = ('phosphate', 'gold', 'gemstone')     # marked, never avoided (mine scenario)
NOT_AVOIDED = RESOURCE_EVENTS + ('searched',)            # + victim-search markers


@dataclass
class Params:
    plan_res: float = 0.10          # planning grid resolution (m)
    # The body is 0.43 x 0.38 m (wheels and camera included). Paths keep the
    # robot centre >= robot_radius from the centre of every occupied map cell
    # (>= 0.26 m from the obstacle itself in the worst case: 0.19 half width +
    # margin, so 0.9 m gaps stay passable). Turning on the spot sweeps a 0.30 m
    # circle: before any turn on the spot the LiDAR checks that circle and the
    # robot first creeps away from anything inside it.
    robot_radius: float = 0.36      # hard clearance for the robot centre (m)
    footprint_radius: float = 0.30  # circle swept when turning on the spot (m)
    soft_radius: float = 0.65       # paths prefer to stay this far from walls (m)
    soft_weight: float = 3.0
    keepout_radius: float = 1.2     # no-go disc around each hazard (m)
    hazard_merge_dist: float = 1.0  # sightings closer than this = same hazard
    retreat_path_margin: float = 0.4   # retreat if the path passes within keepout+this
    retreat_clear: float = 2.5      # retreat until this far from every hazard (m)
    retreat_min: float = 1.5        # ...and at least this far back along the trail
    retreat_max: float = 12.0
    backup_dist: float = 0.30       # straight reverse before turning round
    lookahead: float = 0.70
    v_max: float = 0.70             # m/s (the diff drive brakes at 2 m/s^2: 0.12 m from full speed)
    v_min: float = 0.10
    w_max: float = 1.6              # rad/s
    rotate_in_place: float = 1.0    # heading error (rad) above which we turn on the spot
    rotate_exit: float = 0.5        # ...and below which we drive again (hysteresis: no chattering)
    accel: float = 1.0              # m/s^2: smooth speed-up (braking is immediate)
    corner_slow: float = 0.8        # m/s per m before a sharp corner (0.12 m/s at the corner)
    goal_tolerance: float = 0.30
    frontier_min_cells: int = 5     # min frontier cluster size (planning cells)
    frontier_wall_cells: int = 3    # unknown cells this close to a wall are not frontiers
    goal_search_cells: int = 4      # goal may be this many cells from its frontier
    min_goal_dist: float = 0.45     # frontiers closer than this are considered reached
    blacklist_radius: float = 0.7
    stuck_time: float = 8.0         # s without progress along the path -> recover
    stuck_progress: float = 0.15
    blocked_time: float = 2.0       # s commanded to move while the robot does not move -> recover at once
    recover_speed: float = 0.20     # m/s backing up in a recovery
    emergency_dist: float = 0.30    # LiDAR range in the front sector that stops forward motion
    emergency_sector_deg: float = 35.0
    rear_clear_dist: float = 0.45
    lidar_x: float = 0.10           # LiDAR position ahead of base_footprint
    # robot footprint in the LiDAR frame (points inside = self-hits, ignored)
    self_box: Tuple[float, float, float, float] = (-0.34, 0.13, -0.22, 0.22)
    scan_inject_range: float = 5.0  # add fresh LiDAR hits to the planning grid
    history_step: float = 0.15
    map_wait: float = 2.0           # after reaching a frontier, wait for a newer map (s)
    home_on_finish: bool = True
    # camera coverage: the LiDAR maps a room from the doorway, but hazards are
    # only recognised by the CAMERA (narrow view, ~3.5 m). Every reachable
    # area must therefore also be looked at by the camera before the map is
    # declared finished.
    coverage: bool = True
    camera_x: float = 0.213         # camera ahead of base_footprint (m)
    camera_fov: float = 0.90        # horizontal FOV used for coverage (rad, < real 1.089)
    camera_range: float = 3.2       # hazards are recognised reliably within this (m)
    cov_block: int = 3              # coverage bookkeeping in blocks of 3x3 planning cells
    cov_min_blocks: int = 3         # ignore unseen patches smaller than this many blocks
    cov_penalty: float = 3.0        # m: LiDAR frontiers are preferred over camera-only targets
    cov_view_dist: float = 2.2      # viewpoint must be within this of the unseen patch (m)
    object_max_size: float = 1.2    # isolated obstacles up to this size may be hazards...
    object_penalty: float = 1.0     # ...and are looked at (camera centred on them) first
    look_dwell: float = 0.35        # s facing a suspicious OBJECT so the camera gets frames (6 Hz: 2 frames)
    sweep_dwell: float = 0.25       # s held at an AREA look heading only before turning BACK: turning on,
                                    # the camera sweeps on over the area (0.9 rad coverage cone < 1.09 rad view)
    look_w_max: float = 1.6         # rad/s when turning the camera toward an area
    look_tol: float = 0.25          # rad: an area look needs no precise heading
    look_max: int = 3               # headings per camera stop
    spin_rate: float = 1.0          # rad/s of the full turn at the start (slam_toolbox takes a scan per 0.5 s)
    time_report: float = 60.0       # s between "time so far" lines in the log (0 = never)
    verbose: bool = False           # log every planning decision
    # the Executor plans through space its own map has not seen yet (the
    # beacon route says it is passable); the Writer never does
    unknown_is_free: bool = False
    unknown_cost: float = 1.5
    # slam_toolbox adds a scan to its map only after the robot moved 0.3 m or
    # turned 0.3 rad, and marks a cell only when several rays crossed it: a
    # robot standing still sees a map of barely 1 m around itself. A full turn
    # on the spot first gives SLAM ~20 scans and the camera a look all around.
    initial_spin: bool = True
    map_pad: float = 0.6            # unknown margin around the SLAM map (m): frontiers at its edge count


class Grid:
    """Planning grid built from a SLAM OccupancyGrid (row = y, col = x)."""

    def __init__(self, data: np.ndarray, res: float, ox: float, oy: float, p: Params,
                 scan_pts: Optional[np.ndarray] = None):
        f = max(1, int(round(p.plan_res / res)))
        h, w = data.shape
        H, W = -(-h // f), -(-w // f)
        fine = np.full((H * f, W * f), -1, dtype=np.int16)
        fine[:h, :w] = data
        occ_f = fine >= 50
        free_f = (fine >= 0) & (fine < 50)
        occ = occ_f.reshape(H, f, W, f).any(axis=(1, 3))
        nfree = free_f.reshape(H, f, W, f).sum(axis=(1, 3))
        self.res = res * f
        self.ox, self.oy = ox, oy
        self.H, self.W = H, W
        self.fine = data
        self.fine_res = res
        if scan_pts is not None and len(scan_pts):
            j = np.floor((scan_pts[:, 0] - ox) / self.res).astype(int)
            i = np.floor((scan_pts[:, 1] - oy) / self.res).astype(int)
            ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
            occ[i[ok], j[ok]] = True
        free = ~occ & (nfree * 2 >= f * f)
        self.occ = occ
        self.free = free
        self.unknown = ~occ & ~free
        # exact Euclidean distance (in cells) to the nearest occupied cell,
        # capped at kmax (no scipy needed)
        kmax = int(math.ceil(p.soft_radius / self.res)) + 1
        dist = edt_capped(occ, kmax)
        self.dist = dist
        # UNKNOWN space worth exploring: unknown cells that are not just the
        # unseen inside of a wall or the shadow right behind an obstacle
        # (i.e. at least `frontier_wall_cells` from any occupied cell)
        self.unknown_open = self.unknown & (dist >= p.frontier_wall_cells)
        k_hard = p.robot_radius / self.res
        if p.unknown_is_free:
            self.trav = ~occ & (dist >= k_hard)
        else:
            self.trav = free & (dist >= k_hard)
        soft = np.clip((kmax - dist) / max(1.0, kmax - k_hard), 0.0, 1.0)
        self.cost = 1.0 + p.soft_weight * soft ** 2
        if p.unknown_is_free:
            self.cost = np.where(self.unknown, self.cost * p.unknown_cost, self.cost)

    # coordinates
    def cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def center(self, i, j):
        return self.ox + (j + 0.5) * self.res, self.oy + (i + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.H and 0 <= j < self.W

    def disc_mask(self, pts, radius):
        m = np.zeros((self.H, self.W), dtype=bool)
        if not pts:
            return m
        ys = self.oy + (np.arange(self.H) + 0.5) * self.res
        xs = self.ox + (np.arange(self.W) + 0.5) * self.res
        for (x, y) in pts:
            m |= ((xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2) <= radius * radius
        return m

    def unknown_near(self, x, y, r):
        """Number of open-unknown planning cells within r of (x, y)."""
        i0, j0 = self.cell(x - r, y - r)
        i1, j1 = self.cell(x + r, y + r)
        i0, j0 = max(i0, 0), max(j0, 0)
        i1, j1 = min(i1 + 1, self.H), min(j1 + 1, self.W)
        if i1 <= i0 or j1 <= j0:
            return 0
        return int(self.unknown_open[i0:i1, j0:j1].sum())


_EDT_OFFSETS = {}


def edt_capped(occ: np.ndarray, kmax: int) -> np.ndarray:
    """Euclidean distance (cells, between cell centres) from every cell to the
    nearest True cell of `occ`; kmax + 1 where none is within kmax."""
    H, W = occ.shape
    offs = _EDT_OFFSETS.get(kmax)
    if offs is None:
        offs = sorted(((di * di + dj * dj, di, dj) for di in range(-kmax, kmax + 1)
                       for dj in range(-kmax, kmax + 1) if di * di + dj * dj <= kmax * kmax))
        _EDT_OFFSETS[kmax] = offs
    big = float((kmax + 1) ** 2)
    d2 = np.full((H, W), big, dtype=np.float32)
    pad = np.zeros((H + 2 * kmax, W + 2 * kmax), dtype=bool)
    pad[kmax:kmax + H, kmax:kmax + W] = occ
    unset = np.ones((H, W), dtype=bool)
    for r2, di, dj in offs:                 # nearest offsets first
        sh = pad[kmax + di:kmax + di + H, kmax + dj:kmax + dj + W]
        m = sh & unset
        if m.any():
            d2[m] = r2
            unset &= ~m
    return np.sqrt(d2)


def _dilate4(m):
    o = m.copy()
    o[1:, :] |= m[:-1, :]
    o[:-1, :] |= m[1:, :]
    o[:, 1:] |= m[:, :-1]
    o[:, :-1] |= m[:, 1:]
    return o


def _dilate8(m):
    o = _dilate4(m)
    o[1:, 1:] |= m[:-1, :-1]
    o[1:, :-1] |= m[:-1, 1:]
    o[:-1, 1:] |= m[1:, :-1]
    o[:-1, :-1] |= m[1:, 1:]
    return o


def dijkstra(trav: np.ndarray, cost: np.ndarray, start: Tuple[int, int]):
    """Cheapest-path distances from start over traversable cells (8-connected).
    Returns (dist, parent) as padded flat lists plus the padded width."""
    H, W = trav.shape
    W2 = W + 2
    tp = np.zeros((H + 2, W + 2), dtype=bool)
    tp[1:-1, 1:-1] = trav
    cp = np.ones((H + 2, W + 2))
    cp[1:-1, 1:-1] = cost
    tl = tp.ravel().tolist()
    cl = cp.ravel().tolist()
    n = (H + 2) * W2
    dist = [INF] * n
    par = [-1] * n
    s = (start[0] + 1) * W2 + start[1] + 1
    dist[s] = 0.0
    heap = [(0.0, s)]
    nbs = ((1, 1.0), (-1, 1.0), (W2, 1.0), (-W2, 1.0),
           (W2 + 1, SQ2), (W2 - 1, SQ2), (-W2 + 1, SQ2), (-W2 - 1, SQ2))
    pop, push = heapq.heappop, heapq.heappush
    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue
        cu = cl[u]
        for off, wgt in nbs:
            v = u + off
            if not tl[v]:
                continue
            nd = d + wgt * 0.5 * (cu + cl[v])
            if nd < dist[v]:
                dist[v] = nd
                par[v] = u
                push(heap, (nd, v))
    return dist, par, W2


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Hazard:
    kind: str
    x: float
    y: float
    n: int = 1
    first_seen: float = 0.0


class ExplorerCore:
    LOG_TAG = 'explorer'

    def __init__(self, params: Optional[Params] = None, log=print):
        self.p = params or Params()
        self.log = log
        self.state = 'WAIT'
        self.map = None          # (data, res, ox, oy, stamp)
        self.map_stamp = -1.0
        self.grid: Optional[Grid] = None
        self.grid_stamp = -2.0
        self.pose = None         # (x, y, yaw)
        self.scan = None         # (angles, ranges) filtered, LiDAR frame
        self.scan_xy = None      # LiDAR hits in the LiDAR frame
        self.front_min = INF
        self.rear_min = INF
        self.near_d = INF
        self.near_a = 0.0
        self.hazards: List[Hazard] = []
        self.home = None
        self.history: List[Tuple[float, float]] = []
        self.blacklist: List[Tuple[float, float]] = []
        self.goal_fail = {}
        self.path: List[Tuple[float, float]] = []
        self.path_i = 0
        self.goal = None
        self.goal_frontier = None
        self.goal_kind = None    # 'frontier' | 'home'
        self.t_state = 0.0
        self.progress_best = INF
        self.progress_t = 0.0
        self.emergency_since = None
        self.emergency_count = 0
        self.v_prev = 0.0             # last forward command (acceleration ramp)
        self.turning = False          # rotate-in-place hysteresis
        self.w_prev = 0.0             # last turn rate command (smoothing)
        self.backup_start = None
        self.recover_phase = 0
        self.recover_turn = 1.0
        self.reached_t = None
        self.arrived_frontier = None
        self.done_checked = False
        self.complete_logged = False
        self.home_fail = 0
        self.scan_pose = None
        self.plan_request = 'initial'
        self.events: List[Tuple[float, str]] = []   # (time, text) for logs / tests
        self.status = 'waiting for map and pose'
        self.retreat_count = 0
        self.plans = 0
        # camera coverage memory, fixed to the map frame (independent of the
        # SLAM map growing): 1600 x 1600 cells of plan_res = +-80 m
        self.CN = 1600
        self.cam_seen = np.zeros((self.CN, self.CN), dtype=bool)
        self.look_at = None        # (x, y) the camera must look at (view goals)
        self.look_heading = None
        self.look_dwell_t = None
        self.look_n0 = 0
        self.look_count = 0
        self.look_object = False
        self.objects = []          # small isolated obstacles: (cx, cy, i0, i1, j0, j1)
        self.verified = []         # object positions the camera has looked at
        self.spin_queue = []       # headings of a full turn on the spot
        self.spinning = False
        self.spins = 0             # extra turns because nothing was reachable yet
        self.travelled = 0.0       # metres driven since the start
        self.last_xy = None
        self.spin_turned = 0.0     # heading change the TF saw during the current turn (rad)
        self.spin_last_yaw = None
        self.nudges = 0            # short drives to make SLAM add scans (start only)
        self.nudge_phase = 0
        self.nudge_dir = 1.0
        self.nudge_start = None
        self.nudge_clear = 0.0
        self.warned_yaw = False
        self.dt = 0.1                 # s since the previous control tick (ramps use it)
        self.last_step_t = None
        self.block_ref = None         # (t, x, y, yaw) where the robot last really moved
        self.state_time = {}          # s spent per state (log: "time so far")
        self.recover_why = {}         # recoveries per reason
        self.next_report = None
        self.v_why = ''               # what limits the forward speed right now
        self.plan_pick = None         # point chosen by _plan(targets=...)
        self.look_dir = 0.0           # direction of the current camera turn (+1 left, -1 right)
        self.look_hold_until = None   # hold still until then before turning back

    # ------------------------------------------------------------ inputs
    def set_map(self, data: np.ndarray, res: float, ox: float, oy: float, stamp: float):
        self.map = (data, res, ox, oy)
        self.map_stamp = stamp

    def set_pose(self, x: float, y: float, yaw: float):
        self.pose = (x, y, yaw)
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < 1.0:                       # ignore jumps (SLAM corrections)
                self.travelled += d
        self.last_xy = (x, y)
        if self.home is None:
            self.home = (x, y)
        if not self.history or math.hypot(x - self.history[-1][0], y - self.history[-1][1]) >= self.p.history_step:
            self.history.append((x, y))
            if len(self.history) > 4000:
                self.history = self.history[-3000:]

    def set_scan(self, ranges, angle_min: float, angle_inc: float, range_min: float, range_max: float,
                 pose=None):
        """LiDAR scan (LiDAR frame). `pose` = robot pose (x, y, yaw) when the
        scan was taken (used to put fresh hits into the planning grid)."""
        self.scan_pose = pose
        r = np.asarray(ranges, dtype=float)
        a = angle_min + angle_inc * np.arange(r.size)
        ok = np.isfinite(r) & (r >= max(range_min, 0.05)) & (r <= range_max * 0.999)
        r, a = r[ok], a[ok]
        x, y = r * np.cos(a), r * np.sin(a)
        x0, x1, y0, y1 = self.p.self_box
        mine = (x > x0) & (x < x1) & (y > y0) & (y < y1)
        r, a, x, y = r[~mine], a[~mine], x[~mine], y[~mine]
        self.scan = (a, r)
        self.scan_xy = np.stack([x, y], axis=1)
        half = math.radians(self.p.emergency_sector_deg)
        aw = np.arctan2(np.sin(a), np.cos(a))
        fr = r[np.abs(aw) <= half]
        self.front_min = float(fr.min()) if fr.size else INF
        rr = r[np.abs(aw) >= math.radians(145)]
        self.rear_min = float(rr.min()) if rr.size else INF
        # nearest obstacle point to the robot centre (base frame)
        if x.size:
            bx, by = x + self.p.lidar_x, y
            bd = np.hypot(bx, by)
            k = int(np.argmin(bd))
            self.near_d, self.near_a = float(bd[k]), float(math.atan2(by[k], bx[k]))
        else:
            self.near_d, self.near_a = INF, 0.0

    def add_hazard(self, kind: str, x: float, y: float, now: float) -> bool:
        """A hazard sighting (map frame). Returns True if it is a NEW hazard.
        Resources (mine scenario: phosphate seams, mineral finds) are marked with a
        beacon by the beacon node but are never a keep-out zone: ignored here."""
        if str(kind).lower() in NOT_AVOIDED:
            return False
        for h in self.hazards:
            if h.kind == kind and math.hypot(h.x - x, h.y - y) < self.p.hazard_merge_dist:
                w = 1.0 / (h.n + 1)
                h.x += (x - h.x) * w
                h.y += (y - h.y) * w
                h.n = min(h.n + 1, 30)
                return False
        new = Hazard(kind, x, y, 1, now)
        self.hazards.append(new)
        self._event(now, f'new hazard: {kind} at ({x:.2f}, {y:.2f})')
        if self.pose is not None and self.state not in ('WAIT', 'DONE'):
            rx, ry, _ = self.pose
            d = math.hypot(rx - x, ry - y)
            if self.state == 'RETREAT':
                # already turning back: only react if the trail leads into the new zone
                if d >= self.p.keepout_radius and self._path_hits_keepout(0.0, [new]):
                    self._request_plan(now, 'new hazard on the way back - replanning')
                return True
            in_way = d < self.p.keepout_radius + self.p.retreat_path_margin + 0.2 or \
                self._path_hits_keepout(self.p.retreat_path_margin, [new])
            if in_way:
                self._start_retreat(now, f'{kind} ahead ({d:.1f} m)')
            elif self._path_hits_keepout(0.0, [new]):
                self._request_plan(now, 'path crosses the new keep-out zone')
        return True

    # ------------------------------------------------------------ helpers
    def _event(self, now, text):
        self.events.append((now, text))
        self.log(f'[{self.LOG_TAG}] {text}')

    def _set_state(self, s, now):
        if s != self.state:
            if self.state == 'LOOK':
                self.spinning = False
            self.state = s
            self.t_state = now
            self.turning = False
            self.w_prev = 0.0
            self.block_ref = None
            if s == 'DONE' and self.state_time:
                self.log(f'[{self.LOG_TAG}] ' + self.time_summary().replace('time so far', 'total time'))

    def _tick(self, now):
        """Bookkeeping at the start of every control tick: the tick interval
        (for the speed ramps), the time spent per state and, every
        `time_report` s, a "time so far" line in the log."""
        if self.last_step_t is not None:
            dt = now - self.last_step_t
            self.dt = min(0.3, max(0.02, dt))
            if 0.0 < dt < 1.0:
                self.state_time[self.state] = self.state_time.get(self.state, 0.0) + dt
        self.last_step_t = now
        if self.p.time_report > 0 and self.state not in ('WAIT', 'DONE'):
            if self.next_report is None:
                self.next_report = now + self.p.time_report
            elif now >= self.next_report:
                self.next_report = now + self.p.time_report
                self.log(f'[{self.LOG_TAG}] ' + self.time_summary())

    def time_summary(self):
        """One line: where the (simulated) time went so far."""
        tot = sum(self.state_time.values())
        names = {'FOLLOW': 'driving', 'NAV': 'driving', 'LOOK': 'camera looks', 'PLAN': 'planning',
                 'RECOVER': 'recovering', 'RETREAT': 'retreating', 'NUDGE': 'short drives', 'HOME': 'going home',
                 'TREAT': 'treating'}
        parts = [f'{names[k]} {self.state_time[k]:.0f} s' for k in names if self.state_time.get(k, 0.0) >= 0.5]
        txt = f'time so far {tot / 60:.1f} min: ' + ', '.join(parts) + f'; driven {self.travelled:.0f} m'
        if self.recover_why:
            txt += '; recoveries: ' + ', '.join(f'{n}x {k}' for k, n in self.recover_why.items())
        return txt

    def _blocked(self, now, v, w):
        """True when the robot has been told to move for `blocked_time` s but
        has not moved (stuck on something the LiDAR does not see, wheels
        slipping): recover at once instead of waiting `stuck_time` s."""
        x, y, yaw = self.pose
        trying = abs(v) >= 0.07 or abs(w) >= 0.3
        if not trying or self.block_ref is None:
            self.block_ref = (now, x, y, yaw) if trying else None
            return False
        t0, x0, y0, yaw0 = self.block_ref
        if math.hypot(x - x0, y - y0) > 0.05 or abs(wrap(yaw - yaw0)) > 0.12:
            self.block_ref = (now, x, y, yaw)
            return False
        return now - t0 > self.p.blocked_time

    def _replan_moving(self, now, why):
        """The current goal became useless while driving: pick the next one
        without stopping (keep FOLLOW); stop and plan only if nothing is found."""
        if self.state == 'FOLLOW' and self._plan(now):
            self.emergency_count = 0
            if self.p.verbose:
                self.log(f'[explorer] replanned on the move: {why}')
            return
        self._request_plan(now, why)

    def _request_plan(self, now, why):
        if self.p.verbose:
            self.log(f'[explorer] replan: {why}')
        self.plan_request = why
        self.path = []
        self._set_state('PLAN', now)

    def _hazard_pts(self):
        return [(h.x, h.y) for h in self.hazards]

    def _min_hazard_dist(self, x, y):
        return min((math.hypot(h.x - x, h.y - y) for h in self.hazards), default=INF)

    def _path_hits_keepout(self, margin=0.0, hazards=None):
        """Does the rest of the path enter a keep-out zone (+ margin)?
        Zones the robot is already inside are ignored: the planner only lets
        it move away from their hazard."""
        hazards = self.hazards if hazards is None else hazards
        if not self.path or not hazards or self.pose is None:
            return False
        rx, ry = self.pose[:2]
        r = self.p.keepout_radius + margin
        hz = [(h.x, h.y) for h in hazards if math.hypot(h.x - rx, h.y - ry) >= self.p.keepout_radius]
        if not hz:
            return False
        pts = self.path[self.path_i:]
        prev = (rx, ry)
        for q in pts:
            for t in np.linspace(0.0, 1.0, max(2, int(math.hypot(q[0] - prev[0], q[1] - prev[1]) / 0.1) + 1)):
                x = prev[0] + (q[0] - prev[0]) * t
                y = prev[1] + (q[1] - prev[1]) * t
                if any(math.hypot(x - hx, y - hy) < r for hx, hy in hz):
                    return True
            prev = q
        return False

    def _build_grid(self, with_scan=True):
        data, res, ox, oy = self.map
        if self.p.map_pad > 0 and not self.p.unknown_is_free:
            # beyond the edge of the SLAM map is unknown space: pad it so that
            # free cells on the map edge are frontiers
            k = int(math.ceil(self.p.map_pad / res / 2.0)) * 2
            h, w = data.shape
            big = np.full((h + 2 * k, w + 2 * k), -1, dtype=data.dtype)
            big[k:k + h, k:k + w] = data
            data, ox, oy = big, ox - k * res, oy - k * res
        pts = None
        if with_scan and self.scan_xy is not None and self.pose is not None and len(self.scan_xy):
            rx, ry, yaw = self.scan_pose if self.scan_pose is not None else self.pose
            lx = rx + self.p.lidar_x * math.cos(yaw)
            ly = ry + self.p.lidar_x * math.sin(yaw)
            sel = np.hypot(self.scan_xy[:, 0], self.scan_xy[:, 1]) < self.p.scan_inject_range
            loc = self.scan_xy[sel]
            c, s = math.cos(yaw), math.sin(yaw)
            pts = np.stack([lx + c * loc[:, 0] - s * loc[:, 1], ly + s * loc[:, 0] + c * loc[:, 1]], axis=1)
        self.grid = Grid(data, res, ox, oy, self.p, pts)
        self.grid_stamp = self.map_stamp
        return self.grid

    def _is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.p.blacklist_radius for bx, by in self.blacklist)

    # ------------------------------------------------------------ camera coverage
    def _widx(self, x, y):
        r = self.p.plan_res
        return (np.floor(np.asarray(y) / r).astype(int) + self.CN // 2,
                np.floor(np.asarray(x) / r).astype(int) + self.CN // 2)

    def _cone(self, g, x, y, heading):
        """Planning-grid samples the camera sees from base pose (x, y, heading):
        a cone of camera_fov x camera_range, stopped by occupied cells.
        Returns (px, py, i, j) of the visible samples."""
        p = self.p
        cx, cy = x + p.camera_x * math.cos(heading), y + p.camera_x * math.sin(heading)
        angs = heading + np.linspace(-p.camera_fov / 2, p.camera_fov / 2, 25)
        rs = np.arange(0.0, p.camera_range, g.res * 0.5)
        px = cx + rs[None, :] * np.cos(angs)[:, None]
        py = cy + rs[None, :] * np.sin(angs)[:, None]
        i = np.floor((py - g.oy) / g.res).astype(int)
        j = np.floor((px - g.ox) / g.res).astype(int)
        ins = (i >= 0) & (i < g.H) & (j >= 0) & (j < g.W)
        ic, jc = np.clip(i, 0, g.H - 1), np.clip(j, 0, g.W - 1)
        # only credit what is certainly visible: a ray stops at an obstacle and
        # also at unknown space (e.g. the unmapped inside of a wall behind a
        # gap in its mapped face)
        blocked = ~ins | g.occ[ic, jc] | g.unknown[ic, jc]
        vis = ~np.logical_or.accumulate(blocked, axis=1)
        return px[vis], py[vis], i[vis], j[vis]

    def _mark_camera(self):
        """Mark what the camera sees now."""
        if self.grid is None or self.pose is None or not self.p.coverage:
            return
        x, y, yaw = self.pose
        px, py, _, _ = self._cone(self.grid, x, y, yaw)
        iw, jw = self._widx(px, py)
        ok = (iw >= 0) & (iw < self.CN) & (jw >= 0) & (jw < self.CN)
        self.cam_seen[iw[ok], jw[ok]] = True

    def _mark_seen_disc(self, x, y, r):
        n = int(math.ceil(r / self.p.plan_res))
        i0, j0 = self._widx(x, y)
        ii, jj = np.mgrid[-n:n + 1, -n:n + 1]
        m = ii * ii + jj * jj <= n * n
        a, b = np.clip(i0 + ii[m], 0, self.CN - 1), np.clip(j0 + jj[m], 0, self.CN - 1)
        self.cam_seen[a, b] = True

    def _cam_seen_local(self, g):
        xs = g.ox + (np.arange(g.W) + 0.5) * g.res
        ys = g.oy + (np.arange(g.H) + 0.5) * g.res
        iw, _ = self._widx(0.0 * ys, ys)
        _, jw = self._widx(xs, 0.0 * xs)
        iw = np.clip(iw, 0, self.CN - 1)
        jw = np.clip(jw, 0, self.CN - 1)
        return self.cam_seen[np.ix_(iw, jw)]

    def _unseen(self, g, keep_ext):
        """Free cells the camera has not looked at yet (not in keep-out zones)."""
        return g.free & (g.dist >= 2) & ~self._cam_seen_local(g) & ~keep_ext

    def _best_view(self, g, keep_ext, D, pose):
        """Best camera viewpoint: (score, goal_cell, look_point) or None."""
        p = self.p
        want = self._unseen(g, keep_ext)
        if not want.any():
            return None
        B = p.cov_block
        Hb, Wb = -(-g.H // B), -(-g.W // B)
        wp = np.zeros((Hb * B, Wb * B), dtype=bool)
        wp[:g.H, :g.W] = want
        blocks = wp.reshape(Hb, B, Wb, B).sum(axis=(1, 3)) >= 3
        idx = np.argwhere(blocks)
        if idx.size == 0:
            return None
        bset = set(map(tuple, idx.tolist()))
        seen, clusters = set(), []
        for c in bset:
            if c in seen:
                continue
            stack, comp = [c], []
            seen.add(c)
            while stack:
                u = stack.pop()
                comp.append(u)
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        v = (u[0] + di, u[1] + dj)
                        if v in bset and v not in seen:
                            seen.add(v)
                            stack.append(v)
            if len(comp) >= p.cov_min_blocks:
                clusters.append(comp)
        reach = np.isfinite(D)
        k = int(math.ceil(p.cov_view_dist / g.res))
        best = None
        for comp in clusters:
            mb = np.zeros((Hb, Wb), dtype=bool)
            for (a, b) in comp:
                mb[a, b] = True
            mask = np.kron(mb, np.ones((B, B), dtype=bool))[:g.H, :g.W] & want
            cells = np.argwhere(mask)
            if cells.size == 0:
                continue
            # target: the unseen cell nearest to the patch's centre of mass
            cm = cells.mean(axis=0)
            t = cells[int(np.argmin(((cells - cm) ** 2).sum(axis=1)))]
            ti, tj = int(t[0]), int(t[1])
            tx, ty = g.center(ti, tj)
            if self._is_blacklisted(tx, ty):
                continue
            # viewpoint: reachable, close to the target, clear line of sight
            i0, i1 = max(ti - k, 0), min(ti + k + 1, g.H)
            j0, j1 = max(tj - k, 0), min(tj + k + 1, g.W)
            ii, jj = np.mgrid[i0:i1, j0:j1]
            dt = np.hypot(ii - ti, jj - tj) * g.res
            ok = reach[i0:i1, j0:j1] & (dt <= p.cov_view_dist)
            if not ok.any():
                continue
            ci, cj, cd = ii[ok], jj[ok], dt[ok]
            cost = D[ci, cj] * g.res + cd
            for n in np.argsort(cost)[:40]:
                a, b = int(ci[n]), int(cj[n])
                if not self._line_free(g, (a, b), (ti, tj)):
                    continue
                score = float(cost[n]) + p.cov_penalty - 0.01 * min(len(cells), 200)
                if best is None or score < best[0]:
                    best = (score, (a, b), (tx, ty))
                break
        return best

    def _best_heading(self, g, keep_ext):
        """(heading, n) turning on the spot that shows the camera the most
        unseen cells (n of them), line of sight included."""
        x, y, yaw = self.pose
        want = self._unseen(g, keep_ext)
        best, best_s = (None, 0), 0.0
        if not want.any():
            return best
        for h in yaw + np.linspace(0.0, 2 * math.pi, 24, endpoint=False):
            _, _, i, j = self._cone(g, x, y, h)
            sel = want[i, j]
            if not sel.any():
                continue
            n = np.unique(i[sel] * g.W + j[sel]).size
            # a little preference for small turns (and the turn sweeps the camera anyway)
            sc = n * (1.0 - 0.25 * abs(wrap(h - yaw)) / math.pi)
            if sc > best_s:
                best, best_s = (float(wrap(h)), int(n)), sc
        return best

    def _unseen_near(self, g, x, y, r):
        keep_ext = g.disc_mask(self._hazard_pts(), self.p.keepout_radius + 0.3)
        want = self._unseen(g, keep_ext)
        i, j = g.cell(x, y)
        n = int(math.ceil(r / g.res))
        i0, i1, j0, j1 = max(i - n, 0), min(i + n + 1, g.H), max(j - n, 0), min(j + n + 1, g.W)
        if i1 <= i0 or j1 <= j0:
            return 0
        return int(want[i0:i1, j0:j1].sum())

    # ------------------------------------------------------------ objects
    def _find_objects(self, g):
        """Small isolated obstacles in the map (rubble, hazard blocks...): the
        camera must look at each one, because the LiDAR cannot tell a hazard
        block from rubble. Obstacles closer than ~0.2 m to each other are
        merged (wall pieces merge into walls, which are too big to count)."""
        p = self.p
        m = _dilate8(g.occ)
        H, W = m.shape
        flat = m.ravel()
        seen = np.zeros(H * W, dtype=bool)
        occ_flat = g.occ.ravel()
        objects = []
        maxc = int(p.object_max_size / g.res) + 3
        for start in np.flatnonzero(flat):
            if seen[start]:
                continue
            seen[start] = True
            stack = [int(start)]
            comp = []
            big = False
            while stack:
                u = stack.pop()
                comp.append(u)
                ui, uj = divmod(u, W)
                for di in (-1, 0, 1):
                    vi = ui + di
                    if vi < 0 or vi >= H:
                        continue
                    for dj in (-1, 0, 1):
                        vj = uj + dj
                        if vj < 0 or vj >= W:
                            continue
                        v = vi * W + vj
                        if flat[v] and not seen[v]:
                            seen[v] = True
                            stack.append(v)
                if len(comp) > maxc * maxc * 2:
                    big = True
            if big:
                continue
            cells = np.array([c for c in comp if occ_flat[c]])
            if cells.size < 2:
                continue
            ci, cj = cells // W, cells % W
            i0, i1, j0, j1 = int(ci.min()), int(ci.max()), int(cj.min()), int(cj.max())
            if max(i1 - i0 + 1, j1 - j0 + 1) * g.res > p.object_max_size:
                continue
            cx = g.ox + (cj.mean() + 0.5) * g.res
            cy = g.oy + (ci.mean() + 0.5) * g.res
            objects.append((float(cx), float(cy), i0, i1, j0, j1))
        return objects

    def _object_done(self, ob):
        cx, cy = ob[0], ob[1]
        if any(math.hypot(cx - vx, cy - vy) < 0.5 for vx, vy in self.verified):
            return True
        return any(math.hypot(cx - h.x, cy - h.y) < 1.0 for h in self.hazards)

    def _sees_object(self, g, x0, y0, ob):
        """Clear line of sight from (x0, y0) to the object (stops at it)."""
        cx, cy, i0, i1, j0, j1 = ob
        L = math.hypot(cx - x0, cy - y0)
        for t in np.arange(0.0, L, g.res * 0.5):
            x = x0 + (cx - x0) * t / L
            y = y0 + (cy - y0) * t / L
            i, j = g.cell(x, y)
            if i0 - 1 <= i <= i1 + 1 and j0 - 1 <= j <= j1 + 1:
                return True
            if not g.inside(i, j) or g.occ[i, j] or g.unknown[i, j]:
                return False
        return True

    def _verify_objects(self):
        """Objects centred in the camera view, in range and in sight = looked at."""
        g, p = self.grid, self.p
        if g is None or not self.objects:
            return
        x, y, yaw = self.pose
        cx, cy = x + p.camera_x * math.cos(yaw), y + p.camera_x * math.sin(yaw)
        for ob in self.objects:
            d = math.hypot(ob[0] - cx, ob[1] - cy)
            if d > p.camera_range or d < 0.3 or self._object_done(ob):
                continue
            if abs(wrap(math.atan2(ob[1] - cy, ob[0] - cx) - yaw)) > p.camera_fov / 2 - 0.1:
                continue
            if self._sees_object(g, cx, cy, ob):
                self.verified.append((ob[0], ob[1]))

    def _best_object_view(self, g, keep, D):
        """Viewpoint for the cheapest object not looked at yet:
        (score, goal_cell, object_centre) or None."""
        p = self.p
        reach = np.isfinite(D)
        rmax = p.camera_range - 0.3
        k = int(math.ceil(rmax / g.res))
        best = None
        for ob in self.objects:
            if self._object_done(ob) or self._is_blacklisted(ob[0], ob[1]):
                continue
            ti, tj = g.cell(ob[0], ob[1])
            if not g.inside(ti, tj) or keep[ti, tj]:
                continue
            i0, i1 = max(ti - k, 0), min(ti + k + 1, g.H)
            j0, j1 = max(tj - k, 0), min(tj + k + 1, g.W)
            ii, jj = np.mgrid[i0:i1, j0:j1]
            dt = np.hypot(ii - ti, jj - tj) * g.res
            ok = reach[i0:i1, j0:j1] & (dt >= 0.9) & (dt <= rmax)
            if not ok.any():
                continue
            ci, cj = ii[ok], jj[ok]
            cost = D[ci, cj] * g.res
            for n in np.argsort(cost)[:30]:
                a, b = int(ci[n]), int(cj[n])
                vx, vy = g.center(a, b)
                if not self._sees_object(g, vx, vy, ob):
                    continue
                score = float(cost[n]) + p.object_penalty
                if best is None or score < best[0]:
                    best = (score, (a, b), (ob[0], ob[1]))
                break
        return best

    # ------------------------------------------------------------ planning
    def _plan(self, now, target=None, targets=None):
        """Plan to the best frontier (target None), to a given point, or to
        the best of several points `targets` [(x, y, extra_cost_m), ...]
        (the cheapest reachable one: path length + extra cost; chosen point
        in self.plan_pick). Returns True if a path was set."""
        g = self._build_grid()
        self.plans += 1
        rx, ry, ryaw = self.pose
        si, sj = g.cell(rx, ry)
        R = self.p.keepout_radius
        keep = g.disc_mask(self._hazard_pts(), R)
        trav = g.trav.copy()
        cost = g.cost.copy()
        if self.hazards:
            xs = g.ox + (np.arange(g.W) + 0.5) * g.res
            ys = g.oy + (np.arange(g.H) + 0.5) * g.res
            for h in self.hazards:
                dh = np.hypot(xs[None, :] - h.x, ys[:, None] - h.y)
                d_robot = math.hypot(rx - h.x, ry - h.y)
                if d_robot < R:
                    # the robot is inside this zone: it may only move AWAY from
                    # the hazard (never closer than it is now), expensively
                    trav &= ~(dh < d_robot - 0.15)
                    cost = cost + 30.0 * (dh < R)
                else:
                    trav &= ~(dh < R)
        start = self._nearest_trav(trav, g, si, sj)
        self.plan_fail = None
        if start is None:
            self.plan_fail = 'start'
            self._event(now, 'no free cell around the robot - recovering')
            return False
        dist, par, W2 = dijkstra(trav, cost, start)

        def d_of(i, j):
            return dist[(i + 1) * W2 + j + 1]

        if targets is not None:
            best = None
            for (tx, ty, extra) in targets:
                ti, tj = g.cell(tx, ty)
                for di in range(-2, 3):
                    for dj in range(-2, 3):
                        i, j = ti + di, tj + dj
                        if g.inside(i, j) and d_of(i, j) < INF:
                            c = d_of(i, j) * g.res + extra + math.hypot(di, dj) * g.res
                            if best is None or c < best[0]:
                                best = (c, i, j, (tx, ty))
            if best is None:
                self.plan_fail = 'target'
                return False
            goal_cell = (best[1], best[2])
            self.plan_pick = best[3]
            self.goal_frontier = None
            self.look_at = None
        elif target is not None:
            ti, tj = g.cell(*target)
            best = None
            for di in range(-6, 7):
                for dj in range(-6, 7):
                    i, j = ti + di, tj + dj
                    if g.inside(i, j) and d_of(i, j) < INF:
                        dd = math.hypot(di, dj)
                        if best is None or dd < best[0]:
                            best = (dd, i, j)
            if best is None:
                self.plan_fail = 'target'
                return False
            goal_cell = (best[1], best[2])
            self.goal_frontier = None
            self.look_at = None
        else:
            fr = self._best_frontier(g, keep, d_of, (rx, ry, ryaw))
            vw = ob = None
            if self.p.coverage:
                D = np.array(dist, dtype=float).reshape(-1, W2)[1:-1, 1:-1]
                keep_ext = g.disc_mask(self._hazard_pts(), self.p.keepout_radius + 0.3)
                ob = self._best_object_view(g, keep, D)
                vw = self._best_view(g, keep_ext, D, (rx, ry, ryaw))
            cands = [c for c in ((fr, 'frontier'), (ob, 'object'), (vw, 'view')) if c[0] is not None]
            if not cands:
                return False
            (score, goal_cell, info), kind = min(cands, key=lambda c: c[0][0])
            self.look_object = kind == 'object'
            if kind == 'frontier':
                self.goal_frontier = info
                self.look_at = None
                self.goal_kind = 'frontier'
            else:
                self.goal_frontier = None
                self.look_at = info
                self.look_n0 = self._unseen_near(g, info[0], info[1], 1.2)
                self.goal_kind = 'view'
        # reconstruct
        cells = []
        u = (goal_cell[0] + 1) * W2 + goal_cell[1] + 1
        s = (start[0] + 1) * W2 + start[1] + 1
        guard = 0
        while u != -1 and guard < 200000:
            cells.append((u // W2 - 1, u % W2 - 1))
            if u == s:
                break
            u = par[u]
            guard += 1
        cells.reverse()
        pts = self._smooth(cells, trav, g)
        self.path = pts
        self.path_i = 0
        self.goal = pts[-1]
        self.progress_best = INF
        self.progress_t = now
        return True

    def _nearest_trav(self, trav, g, si, sj):
        si = min(max(si, 0), g.H - 1)          # robot just outside a stale map
        sj = min(max(sj, 0), g.W - 1)
        if trav[si, sj]:
            return (si, sj)
        best = None
        R = int(math.ceil(0.6 / g.res))
        for di in range(-R, R + 1):
            for dj in range(-R, R + 1):
                i, j = si + di, sj + dj
                if g.inside(i, j) and trav[i, j] and self._line_free(g, (si, sj), (i, j)):
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, i, j)
        return None if best is None else (best[1], best[2])

    def _line_free(self, g, a, b):
        """No OCCUPIED cell on the straight line a->b (cells); False if the
        line leaves the grid."""
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        for t in np.linspace(0, 1, n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if not g.inside(i, j) or g.occ[i, j]:
                return False
        return True

    def _best_frontier(self, g, keep, d_of, pose):
        p = self.p
        front = g.free & _dilate4(g.unknown_open) & ~keep
        idx = np.argwhere(front)
        if idx.size == 0:
            return None
        # 8-connected clusters
        fset = set(map(tuple, idx.tolist()))
        seen = set()
        clusters = []
        for c in fset:
            if c in seen:
                continue
            stack = [c]
            seen.add(c)
            comp = []
            while stack:
                u = stack.pop()
                comp.append(u)
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        v = (u[0] + di, u[1] + dj)
                        if v in fset and v not in seen:
                            seen.add(v)
                            stack.append(v)
            if len(comp) >= p.frontier_min_cells:
                clusters.append(comp)
        R = p.goal_search_cells
        offs = [(di, dj) for di in range(-R, R + 1) for dj in range(-R, R + 1) if di * di + dj * dj <= R * R]
        rx, ry, ryaw = pose
        best = None
        for comp in clusters:
            cand = None
            for (i, j) in comp:
                for di, dj in offs:
                    a, b = i + di, j + dj
                    if not g.inside(a, b) or keep[a, b]:
                        continue
                    d = d_of(a, b)
                    if d == INF:
                        continue
                    if cand is None or d < cand[0]:
                        cand = (d, a, b, i, j)
            if cand is None:
                continue
            d, a, b, fi, fj = cand
            gx, gy = g.center(a, b)
            fx, fy = g.center(fi, fj)
            if self._is_blacklisted(fx, fy) or self._is_blacklisted(gx, gy):
                continue
            if self.travelled < 1.0 and math.hypot(gx - rx, gy - ry) < p.goal_tolerance + 0.1:
                continue          # (start, tiny map) nothing to drive to: see _start_nudge
            dm = d * g.res
            if math.hypot(gx - rx, gy - ry) < p.min_goal_dist and g.unknown_near(fx, fy, 0.3) > 0 and \
                    self.travelled >= 1.0:
                # standing on it and it is still unknown: LiDAR cannot see it
                self.blacklist.append((fx, fy))
                continue
            turn = abs(wrap(math.atan2(gy - ry, gx - rx) - ryaw)) / math.pi
            size_bonus = 0.3 * min(len(comp) * g.res, 3.0)
            score = dm + 1.0 * turn - size_bonus
            if best is None or score < best[0]:
                best = (score, (a, b), (fx, fy))
        return best

    def _smooth(self, cells, trav, g):
        """Line-of-sight shortcutting on the traversable grid."""
        if not cells:
            return []
        out = [cells[0]]
        i = 0
        n = len(cells)
        while i < n - 1:
            j = i + 1
            while j + 1 < n and self._los(trav, cells[i], cells[j + 1]) and \
                    math.hypot(cells[j + 1][0] - cells[i][0], cells[j + 1][1] - cells[i][1]) * g.res < 3.0:
                j += 1
            out.append(cells[j])
            i = j
        return [g.center(i, j) for (i, j) in out]

    @staticmethod
    def _los(trav, a, b):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        for t in np.linspace(0, 1, n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if not trav[i, j]:
                return False
        return True

    def _path_blocked(self):
        """Does the rest of the current path cross non-traversable cells of the
        latest grid (new walls, fresh LiDAR hits) or a keep-out zone?"""
        g = self.grid
        if g is None or not self.path:
            return False
        pts = [self.pose[:2]] + self.path[self.path_i:]
        for a, b in zip(pts[:-1], pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            for t in np.linspace(0, 1, max(2, int(L / 0.1) + 1)):
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                # ignore the first 0.25 m (the robot may sit in an inflated cell)
                if math.hypot(x - self.pose[0], y - self.pose[1]) < 0.25:
                    continue
                i, j = g.cell(x, y)
                if not g.inside(i, j) or g.occ[i, j]:
                    return True
                if g.dist[i, j] < 2:        # hard: within ~0.15 m of a wall
                    return True
        return self._path_hits_keepout() if self.state != 'RETREAT' else False

    # ------------------------------------------------------------ retreat
    def _start_retreat(self, now, why):
        self.retreat_count += 1
        # walk back along the trajectory until well clear of every hazard
        hist = self.history
        back, acc = [], 0.0
        prev = self.pose[:2]
        target_ok = False
        r_keep = self.p.keepout_radius
        inside = [h for h in self.hazards if math.hypot(h.x - prev[0], h.y - prev[1]) < r_keep]
        for q in reversed(hist[:-1]):
            # never walk back INTO another hazard's zone (one found after the
            # robot drove past it); leaving the zone we are in is fine
            if any(math.hypot(h.x - q[0], h.y - q[1]) < r_keep and h not in inside for h in self.hazards):
                break
            acc += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
            back.append(q)
            if acc >= self.p.retreat_min and self._min_hazard_dist(*q) >= self.p.retreat_clear:
                target_ok = True
                break
            if acc >= self.p.retreat_max:
                break
        self._event(now, f'RETREAT: {why} -> returning the way it came ({acc:.1f} m)')
        if len(back) < 2:
            self._request_plan(now, 'retreat: no trail behind, replanning')
            return
        # thin the trail to ~0.3 m spacing for the follower
        pts = [back[0]]
        for q in back[1:]:
            if math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) >= 0.3:
                pts.append(q)
        if pts[-1] != back[-1]:
            pts.append(back[-1])
        self.path = pts
        self.path_i = 0
        self.goal = pts[-1]
        self.goal_kind = 'retreat'
        self.backup_start = self.pose[:2]
        self.progress_best = INF
        self.progress_t = now
        self._set_state('RETREAT', now)
        if not target_ok:
            self._event(now, 'retreat: trail ends before a clear point, will replan from there')

    # ------------------------------------------------------------ control
    def _follow(self, now, reverse_first=False):
        """Pure pursuit along self.path. Returns (v, w, arrived)."""
        p = self.p
        x, y, yaw = self.pose
        if reverse_first and self.backup_start is not None:
            moved = math.hypot(x - self.backup_start[0], y - self.backup_start[1])
            if moved < p.backup_dist and self.rear_min > p.rear_clear_dist and now - self.t_state < 4.0:
                return -0.15, 0.0, False
            self.backup_start = None
        path = self.path
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) < p.goal_tolerance:
            return 0.0, 0.0, True
        # 1. project the robot onto the path (segments k -> k+1, searching a
        #    few segments ahead of the current one; never goes backwards)
        n = len(path)
        best = (math.hypot(path[-1][0] - x, path[-1][1] - y), n - 1, path[-1])
        for k in range(self.path_i, min(n - 1, self.path_i + 8)):
            (ax, ay), (bx, by) = path[k], path[k + 1]
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
            qx, qy = ax + ex * t, ay + ey * t
            d = math.hypot(qx - x, qy - y)
            if d <= best[0] + 1e-9:
                best = (d, k, (qx, qy))
        dproj, k0, P = best
        self.path_i = min(k0, n - 1)
        # 2. remaining length (progress watchdog)
        rem = dproj + math.hypot(path[min(k0 + 1, n - 1)][0] - P[0], path[min(k0 + 1, n - 1)][1] - P[1]) + \
            sum(math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]) for k in range(k0 + 1, n - 1))
        if rem < self.progress_best - p.stuck_progress:
            self.progress_best = rem
            self.progress_t = now
        # 3. lookahead: first point after the projection that is `lookahead`
        #    away - but never past a sharp corner the robot has not reached
        #    yet (pure pursuit would cut the corner, toward the obstacle)
        tx, ty = gx, gy
        corner = False
        A = P
        for k in range(k0, n - 1):
            B = path[k + 1]
            dB = math.hypot(B[0] - x, B[1] - y)
            if k + 2 < n and dB > 0.12:
                C = path[k + 2]
                turn = abs(wrap(math.atan2(C[1] - B[1], C[0] - B[0]) -
                                math.atan2(B[1] - path[k][1], B[0] - path[k][0])))
                if turn > 0.6 and dB < p.lookahead:
                    tx, ty = B
                    corner = True
                    break
            if dB >= p.lookahead:
                # circle / segment intersection on A -> B (A is inside the circle)
                ex, ey = B[0] - A[0], B[1] - A[1]
                fx, fy = A[0] - x, A[1] - y
                qa = ex * ex + ey * ey
                qb = 2 * (fx * ex + fy * ey)
                qc = fx * fx + fy * fy - p.lookahead ** 2
                disc = max(0.0, qb * qb - 4 * qa * qc)
                t = (-qb + math.sqrt(disc)) / (2 * qa) if qa > 1e-12 else 1.0
                t = max(0.0, min(1.0, t))
                tx, ty = A[0] + ex * t, A[1] + ey * t
                break
            A = B
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        # turn on the spot above rotate_in_place, drive again below rotate_exit
        self.turning = abs(err) > (p.rotate_exit if self.turning else p.rotate_in_place)
        if self.turning:
            self.v_prev = 0.0
            v, w = self._turn_cmd(max(-p.w_max, min(p.w_max, 1.8 * err)))
            if abs(w) > 0.0:
                w = math.copysign(max(0.35, abs(w)), w)
            w = self._smooth_w(w)
            return v, w, False
        dgoal = math.hypot(gx - x, gy - y)
        lims = [('top speed', p.v_max),
                ('heading', p.v_max * max(0.0, math.cos(err)) ** 2)]
        if self._stops_at_goal():
            lims.append(('arrival', 0.12 + 0.5 * dgoal))            # gentle arrival
        if corner:                                                  # slow down before a sharp corner
            lims.append(('corner', 0.12 + p.corner_slow * math.hypot(tx - x, ty - y)))
        # never reach the LiDAR stop line fast
        lims.append(('obstacle', 0.10 + 1.0 * max(0.0, self.front_min - p.emergency_dist)))
        lims.append(('speeding up', self.v_prev + p.accel * self.dt))
        self.v_why, v = min(lims, key=lambda q: q[1])
        if v >= 0.97 * p.v_max:
            self.v_why = 'top speed'
        v = max(p.v_min, v)
        self.v_prev = v
        w = self._smooth_w(max(-p.w_max, min(p.w_max, 1.8 * err)))
        return v, w, False

    def _stops_at_goal(self):
        """Does the robot stop at the end of the current path? (The Executor
        drives through intermediate beacons without slowing down.)"""
        return True

    def _smooth_w(self, w, step=0.3):
        """Turn rate changes by at most `step` rad/s per 0.1 s (3 rad/s^2 for 0.3)."""
        step = step * self.dt / 0.1
        w = max(self.w_prev - step, min(self.w_prev + step, w))
        self.w_prev = w
        return w

    def _turn_cmd(self, w):
        """Turn on the spot - unless something is inside the circle the body
        sweeps when turning: then first creep away from it."""
        p = self.p
        if self.near_d < p.footprint_radius + 0.03:
            if abs(self.near_a) < math.pi / 2 and self.rear_min > p.rear_clear_dist:
                return -0.08, 0.0
            if abs(self.near_a) >= math.pi / 2 and self.front_min > p.emergency_dist + 0.05:
                return 0.08, 0.0
        return 0.0, w

    def _safety(self, now, v, w):
        """Emergency stop on the LiDAR. Returns filtered (v, w)."""
        if v > 0.0 and self.front_min < self.p.emergency_dist:
            if self.emergency_since is None:
                self.emergency_since = now
            self.v_prev = 0.0
            return self._turn_cmd(w) if abs(w) > 0.3 else (0.0, 0.0)
        self.emergency_since = None
        return v, w

    def step(self, now: float) -> Tuple[float, float]:
        """One control tick (call at ~10 Hz). Returns (linear, angular) velocity."""
        p = self.p
        if self.map is None or self.pose is None or self.scan is None:
            self.status = 'waiting for map, pose and scan'
            return 0.0, 0.0
        self._tick(now)
        if self.state == 'WAIT':
            self._event(now, 'map and pose received - exploring')
            if p.initial_spin:
                if self.map_stamp != self.grid_stamp:
                    self._build_grid()
                self._start_spin(now, 'first a full turn on the spot so SLAM maps the surroundings')
                return 0.0, 0.0
            self._request_plan(now, 'start')

        # refresh the planning grid on every new map (cheap: numpy only)
        if self.map_stamp != self.grid_stamp:
            self._build_grid()
            if self.state in ('FOLLOW', 'HOME') and self._path_blocked():
                self._request_plan(now, 'path blocked on the updated map')
            elif self.state == 'FOLLOW' and self.goal_kind == 'frontier' and self.goal_frontier is not None and \
                    self.grid.unknown_near(*self.goal_frontier, 0.35) == 0:
                self._replan_moving(now, 'frontier explored')
            elif self.state == 'FOLLOW' and self.goal_kind == 'view' and not self.look_object and \
                    self.look_at is not None and \
                    self._unseen_near(self.grid, self.look_at[0], self.look_at[1], 1.2) < max(6, 0.25 * self.look_n0):
                self._replan_moving(now, 'camera already saw that area on the way')
            if p.coverage:
                self.objects = self._find_objects(self.grid)
        self._mark_camera()
        if p.coverage:
            self._verify_objects()
            if self.state == 'FOLLOW' and self.goal_kind == 'view' and self.look_object and \
                    any(math.hypot(self.look_at[0] - vx, self.look_at[1] - vy) < 0.5 for vx, vy in self.verified):
                self._replan_moving(now, 'camera already looked at that object on the way')

        if self.state == 'PLAN':
            # first tick in PLAN: stop; next tick: plan (the robot is stopped)
            if now - self.t_state < 0.15:
                return 0.0, 0.0
            if self.reached_t is not None and self.map_stamp <= self.reached_t and now - self.reached_t < p.map_wait:
                self.status = 'waiting for a fresh map'
                return 0.0, 0.0
            self.reached_t = None
            if self.arrived_frontier is not None:
                fx, fy = self.arrived_frontier
                self.arrived_frontier = None
                if self.grid is not None and self.grid.unknown_near(fx, fy, 0.25) > 0 and self.travelled >= 1.0:
                    # we stood next to it with a fresh scan and it is still
                    # unknown: the LiDAR cannot see it, do not come back
                    self.blacklist.append((fx, fy))
                    if p.verbose:
                        self.log(f'[explorer] frontier ({fx:.2f}, {fy:.2f}) unobservable - blacklisted')
            if self._plan(now):
                self.emergency_count = 0
                if self.goal_kind == 'view':
                    self.status = f'camera check of ({self.look_at[0]:.1f}, {self.look_at[1]:.1f})'
                    info = f'look at ({self.look_at[0]:.2f}, {self.look_at[1]:.2f})'
                else:
                    self.status = f'going to frontier ({self.goal[0]:.1f}, {self.goal[1]:.1f})'
                    info = f'frontier ({self.goal_frontier[0]:.2f}, {self.goal_frontier[1]:.2f})'
                if p.verbose:
                    self.log(f'[explorer] goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}) {info}, '
                             f'{len(self.path)} waypoints')
                self._set_state('FOLLOW', now)
            else:
                if self.grid is not None and self._nearest_trav(self.grid.trav, self.grid, *self.grid.cell(*self.pose[:2])) is None:
                    self._start_recover(now, 'no free space around the robot')
                    return 0.0, 0.0
                if self.travelled < 1.0 and self.nudges + self.spins < 4:
                    # nothing to do although the robot has hardly moved: the
                    # map is almost certainly still too small, not finished.
                    # slam_toolbox adds scans when the robot MOVES: drive a
                    # little toward open space (LiDAR), else turn once more
                    self.log('[explorer] map check: ' + self._diag())
                    if self._start_nudge(now):
                        return 0.0, 0.0
                    self.spins += 1
                    self._start_spin(now, 'nothing reachable yet but the robot has not moved - '
                                          'turning on the spot to map more')
                    return 0.0, 0.0
                if not self.complete_logged:
                    self.complete_logged = True
                    self._event(now, 'no reachable frontier and nothing left for the camera - EXPLORATION COMPLETE')
                    self.log('[explorer] map check: ' + self._diag())
                if p.home_on_finish and self.home and self.home_fail < 3 and \
                        math.hypot(self.pose[0] - self.home[0], self.pose[1] - self.home[1]) > 0.5 and \
                        self._plan(now, target=self.home):
                    self.goal_kind = 'home'
                    self.status = 'map complete - returning to start'
                    self._set_state('HOME', now)
                else:
                    self._finish(now)
            return 0.0, 0.0

        if self.state == 'DONE':
            self.status = 'EXPLORATION COMPLETE - stopped at start'
            # one late re-check: a newer map may reveal something missed
            if not self.done_checked and now - self.t_state > 12.0:
                self.done_checked = True
                if self._plan(now):
                    self._event(now, 'late re-check found something new - exploring again')
                    self.complete_logged = False
                    self._set_state('FOLLOW', now)
            return 0.0, 0.0

        if self.state == 'LOOK':
            if self.spinning and self.spin_last_yaw is not None:
                self.spin_turned += abs(wrap(self.pose[2] - self.spin_last_yaw))
                self.spin_last_yaw = self.pose[2]
            return self._look(now)

        if self.state == 'NUDGE':
            return self._nudge(now)

        if self.state == 'RECOVER':
            return self._recover(now)

        if self.state in ('FOLLOW', 'HOME', 'RETREAT'):
            v, w, arrived = self._follow(now, reverse_first=(self.state == 'RETREAT'))
            if arrived:
                if self.state == 'RETREAT':
                    self._event(now, 'retreat finished - exploring the rest of the map')
                    self._request_plan(now, 'after retreat')
                elif self.state == 'HOME':
                    self._finish(now)
                elif self.goal_kind == 'view':
                    self._start_look(now)
                else:
                    # checked against the next map: still unknown -> unobservable
                    self.arrived_frontier = self.goal_frontier
                    self.reached_t = now
                    self._request_plan(now, 'goal reached')
                return 0.0, 0.0
            v, w = self._safety(now, v, w)
            if self.emergency_since is not None and now - self.emergency_since > 1.0:
                self.emergency_since = None
                self.emergency_count += 1
                if self.emergency_count >= 3:
                    self.emergency_count = 0
                    self._start_recover(now, 'obstacle keeps blocking the way')
                else:
                    self._request_plan(now, 'obstacle ahead (LiDAR)')
                return 0.0, 0.0
            if now - self.progress_t > p.stuck_time:
                self._start_recover(now, 'no progress')
                return 0.0, 0.0
            if self._blocked(now, v, w):
                self._start_recover(now, 'blocked (told to move, not moving)')
                return 0.0, 0.0
            if self.state == 'RETREAT' and now - self.t_state > 45.0:
                self._request_plan(now, 'retreat timeout')
                return 0.0, 0.0
            label = {'FOLLOW': 'exploring', 'HOME': 'returning to start', 'RETREAT': 'retreating'}[self.state]
            self.status = f'{label}: {len(self.path) - self.path_i} waypoints left'
            return v, w
        return 0.0, 0.0

    # ------------------------------------------------------------ camera look
    def _start_look(self, now):
        self.look_count = 0
        self.look_dir = 0.0
        self.look_hold_until = None
        self._set_state('LOOK', now)
        if self.look_object:
            # face the object itself (one look)
            self.look_heading = math.atan2(self.look_at[1] - self.pose[1], self.look_at[0] - self.pose[0])
            self.look_count = self.p.look_max
            self.look_dwell_t = None
        else:
            self._next_look(now)

    def _start_spin(self, now, why):
        """One continuous full turn on the spot (LOOK state), then plan on a fresh map."""
        self._event(now, why)
        yaw = self.pose[2]
        self.spinning = True
        self.spin_turned = 0.0
        self.spin_last_yaw = yaw
        self.look_at = None
        self.look_object = False
        self.look_count = 0
        self.look_heading = None
        self.path = []
        self._set_state('LOOK', now)

    def _open_direction(self):
        """(angle, clearance) of the most open direction in the current scan
        (LiDAR frame), using the smallest range within +-15 deg of it."""
        if self.scan is None:
            return None
        a, r = self.scan
        if a.size < 10:
            return None
        best = None
        for c in np.radians(np.arange(-180, 180, 10)):
            d = np.abs(np.arctan2(np.sin(a - c), np.cos(a - c)))
            sel = r[d <= math.radians(15)]
            clear = float(sel.min()) if sel.size else 12.0
            clear = min(clear, 12.0)
            score = clear - 0.3 * abs(c)          # prefer ahead when equal
            if best is None or score > best[0]:
                best = (score, float(c), clear)
        return best[1], best[2]

    def _front_clear(self, half_deg=15):
        a, r = self.scan
        sel = r[np.abs(np.arctan2(np.sin(a), np.cos(a))) <= math.radians(half_deg)]
        return float(sel.min()) if sel.size else 12.0

    def _start_nudge(self, now):
        """Drive ~0.6 m toward the most open direction the LiDAR sees, so
        slam_toolbox (which adds scans every 0.3 m) grows its map."""
        if self.nudges >= 3:
            return False
        od = self._open_direction()
        if od is None or od[1] < 1.0:
            return False
        self.nudges += 1
        self.nudge_dir = 1.0 if od[0] >= 0 else -1.0
        self.nudge_clear = od[1]
        self.nudge_phase = 0 if abs(od[0]) > 0.25 else 1
        self.nudge_start = self.pose[:2]
        self.path = []
        self._event(now, f'map still tiny: driving a little toward open space ({od[1]:.1f} m free) so SLAM '
                         f'adds scans')
        self._set_state('NUDGE', now)
        return True

    def _nudge(self, now):
        el = now - self.t_state
        if self.nudge_phase == 0:
            # turn (robot frame, no TF heading needed) until the open side is ahead
            if self._front_clear() >= min(0.8 * self.nudge_clear, 1.5) or el > 8.0:
                self.nudge_phase = 1
                self.nudge_start = self.pose[:2]
                self.t_state = now
                return 0.0, 0.0
            return self._turn_cmd(0.6 * self.nudge_dir)
        moved = math.hypot(self.pose[0] - self.nudge_start[0], self.pose[1] - self.nudge_start[1])
        if moved < 0.6 and el < 8.0 and self._front_clear(25) > 0.55 and self.front_min > self.p.emergency_dist + 0.1:
            return 0.15, 0.0
        self.reached_t = now          # plan on a map made after the drive
        self._request_plan(now, 'after the short drive')
        return 0.0, 0.0

    def _diag(self):
        """One line on why there is (or is not) anything to explore."""
        g = self.grid
        if g is None or self.pose is None:
            return 'no map yet'
        si, sj = g.cell(*self.pose[:2])
        start = self._nearest_trav(g.trav, g, si, sj)
        reach = 0.0
        if start is not None:
            dist, _, W2 = dijkstra(g.trav, g.cost, start)
            reach = float(np.isfinite(np.array(dist, dtype=float)).sum()) * g.res ** 2
        front = int((g.free & _dilate4(g.unknown_open)).sum())
        return (f'SLAM map {self.map[0].shape[1]}x{self.map[0].shape[0]} cells of {self.map[1]:.2f} m, '
                f'free {g.free.sum() * g.res ** 2:.1f} m2, reachable {reach:.1f} m2, frontier cells {front}, '
                f'blacklisted {len(self.blacklist)}, driven {self.travelled:.1f} m')

    def _next_look(self, now):
        """Pick the next heading to look at (greedy, at most `look_max` per stop)."""
        keep_ext = self.grid.disc_mask(self._hazard_pts(), self.p.keepout_radius + 0.3)
        h, n = self._best_heading(self.grid, keep_ext)
        if h is None or n < 6 or self.look_count >= self.p.look_max:
            self.look_heading = None
        else:
            self.look_heading = h
            self.look_count += 1
        self.look_dwell_t = None
        self.t_state = now

    def _spin(self, now):
        """The full turn on the spot: one smooth rotation, no stops (slam_toolbox
        adds a scan every 0.5 s / 0.3 rad, the camera sweeps all around)."""
        p = self.p
        limit = 2.0 * (2 * math.pi / max(0.3, p.spin_rate)) + 4.0
        if self.spin_turned < 2 * math.pi - 0.2 and now - self.t_state < limit:
            self.status = 'full turn on the spot (SLAM maps the surroundings)'
            v, w = self._turn_cmd(p.spin_rate)
            return v, (self._smooth_w(w, 0.3) if w else 0.0)
        self.spinning = False
        self.look_heading = None
        self.reached_t = now                  # plan on a map made after the turn
        if self.spin_turned < 3.0 and not self.warned_yaw:
            self.warned_yaw = True
            self._event(now, f'WARNING: a full turn was commanded but the robot heading (TF map -> '
                             f'base_footprint) changed only {math.degrees(self.spin_turned):.0f} deg - '
                             f'the odometry does not see the turn (run: bash ~/writer_robot_ws/check_sim.sh)')
        self._request_plan(now, 'full turn done')
        return 0.0, 0.0

    def _look(self, now):
        """Turn on the spot so the camera faces the unseen area, then straight
        on to the next heading (an object: face it and hold)."""
        p = self.p
        if self.spinning:
            return self._spin(now)
        for _ in range(2):
            if self.look_heading is None or now - self.t_state >= 10.0:
                break
            err = wrap(self.look_heading - self.pose[2])
            tol = 0.12 if self.look_object else p.look_tol
            if abs(err) > tol and self.look_dwell_t is None:
                if self.look_hold_until is not None and now < self.look_hold_until:
                    self.status = 'looking (camera)'
                    self.w_prev = 0.0
                    return 0.0, 0.0
                self.look_hold_until = None
                self.look_dir = math.copysign(1.0, err)
                self.status = 'turning the camera toward ' + ('an object' if self.look_object else 'an unseen area')
                v, w = self._turn_cmd(math.copysign(max(0.4, min(p.look_w_max, 2.0 * abs(err))), err))
                return v, (self._smooth_w(w, 0.4) if w else 0.0)
            if self.look_dwell_t is None:
                self.look_dwell_t = now
            if self.look_object and now - self.look_dwell_t < p.look_dwell:
                self.status = 'looking (camera)'
                self.w_prev = 0.0
                return 0.0, 0.0
            self._next_look(now)
            if self.look_heading is not None and self.look_dir * wrap(self.look_heading - self.pose[2]) < 0:
                # turning back: first let the camera take frames of the area it just reached
                self.look_hold_until = now + p.sweep_dwell
        if self.look_heading is not None and now - self.t_state < 10.0:
            return 0.0, 0.0                   # (two headings reached in one tick: go on next tick)
        # done: whatever is still hidden around the target cannot be seen from here
        if self.look_at is not None:
            if self.look_object:
                self.verified.append(self.look_at)
            else:
                self._mark_seen_disc(self.look_at[0], self.look_at[1], 0.5)
        self._request_plan(now, 'camera check done')
        return 0.0, 0.0

    def _finish(self, now):
        self._event(now, 'DONE - robot stopped')
        self.path = []
        self._set_state('DONE', now)

    # ------------------------------------------------------------ recovery
    def _start_recover(self, now, why):
        self._event(now, f'recover: {why}')
        if self.goal_kind == 'home':
            self.home_fail += 1
        if self.goal is not None and self.goal_kind in ('frontier', 'view'):
            key = (round(self.goal[0], 1), round(self.goal[1], 1))
            self.goal_fail[key] = self.goal_fail.get(key, 0) + 1
            if self.goal_fail[key] >= 2:
                self.blacklist.append(self.goal)
                if self.goal_frontier:
                    self.blacklist.append(self.goal_frontier)
                if self.goal_kind == 'view' and self.look_at:
                    self.blacklist.append(self.look_at)
                self._event(now, f'goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}) blacklisted')
        key = why.split(' (')[0]
        self.recover_why[key] = self.recover_why.get(key, 0) + 1
        self.recover_phase = 0
        self.backup_start = self.pose[:2]
        # turn toward the more open side
        if self.scan is not None:
            a, r = self.scan
            left = r[(a > 0.3) & (a < 2.0)]
            right = r[(a < -0.3) & (a > -2.0)]
            lm = float(np.median(left)) if left.size else 0.0
            rm = float(np.median(right)) if right.size else 0.0
            self.recover_turn = 1.0 if lm >= rm else -1.0
        self._set_state('RECOVER', now)

    def _recover(self, now):
        x, y, _ = self.pose
        el = now - self.t_state
        if self.recover_phase == 0:
            moved = math.hypot(x - self.backup_start[0], y - self.backup_start[1])
            if moved < 0.3 and self.rear_min > self.p.rear_clear_dist and el < 3.0:
                return -self.p.recover_speed, 0.0
            self.recover_phase = 1
            self.t_state = now
            return 0.0, 0.0
        if self.recover_phase == 1:
            if el < 1.3:
                return self._turn_cmd(1.1 * self.recover_turn)
            self._request_plan(now, 'after recovery')
            return 0.0, 0.0
        return 0.0, 0.0
