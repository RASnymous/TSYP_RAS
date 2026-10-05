"""
executor_core.py - the Executor robot's brain (no ROS inside).

The Executor enters the zone after the Writer. It has no map of its own: all it
knows comes from the Writer's beacons, received over LoRa as signed 44-byte
LMB2 records (kind, position, `next` = the way out). With them it:

  1. builds the BEACON TREE. Every beacon's `next` points to an older beacon,
     all the way back to the EXIT beacon where the Writer came in. Trail and
     exit beacons carry their own position. Hazard records carry the HAZARD's
     position, so the beacon itself (dropped where the Writer stood) is found
     from its `next` vector: drop = parent's drop - next vector.
  2. picks the MISSION TARGETS: every live hazard event (radiation, fire, gas,
     victim ...) whose confidence has not faded away. The order is always the
     nearest next along the tree.
  3. drives to each target BEACON BY BEACON. The route is the tree path: up
     the target's chain of `next` hops, reversed. It plans each short leg on
     its own LiDAR map (unseen space counts as passable: the Writer drove
     there). Every hazard in the records is a 1.2 m keep-out zone.
  4. APPROACHES the hazard to a safe standoff (1.6 m), along the line from
     which the Writer saw it, turns to face it and TREATS it (shielding,
     extinguishing, sealing a leak, first aid: a timed action).
  5. RETURNS to the exit by following `next` hops, and stops: MISSION COMPLETE.

v9 - BRIEFINGS. The Command Post can choose the targets: the operator picks
beacons on the dashboard, the Outside Network Area signs a MISSION frame and
its three gateways transmit it; ona_link_node checks the signature and hands
it over (set_briefing). A briefing gives the targets IN ORDER (hazards are
treated, trail or exit beacons are just visited), may say "don't come back to
the exit", and may ABORT the mission (straight back to the exit). It can
arrive before the start (wait_briefing > 0: the Executor waits for it), or at
any time during the mission: the Executor re-targets on the spot. The robot
acknowledges in its position reports (ROBOT frames, via ona_link_node).

Navigation (path planning, pure pursuit, keep-out zones, turning back when an
unknown hazard appears in its way, recovery, LiDAR safety) is the Writer's
ExplorerCore, reused as it is.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from writer_robot.explorer_core import ExplorerCore, Params, Hazard, wrap, INF

KEEPOUT_KINDS = ('RADIATION', 'THERMAL', 'GAS', 'STRUCTURAL')
CLOSE_KINDS = ('VICTIM', 'PHOSPHATE', 'GOLD', 'GEMSTONE')     # approached to victim_standoff, not standoff
RESOURCE_KINDS = ('PHOSPHATE', 'GOLD', 'GEMSTONE')            # targets only when the Command Post briefs them
MARKER_KINDS = RESOURCE_KINDS + ('SEARCHED',)                  # never a keep-out zone
TASKS = {'RADIATION': 'placing radiation shielding', 'THERMAL': 'extinguishing the fire',
         'GAS': 'sealing the gas leak', 'VICTIM': 'delivering first aid',
         'STRUCTURAL': 'shoring the structure', 'OBSTRUCTION': 'clearing the obstruction',
         'PHOSPHATE': 'taking a phosphate sample', 'GOLD': 'taking a mineral sample for assay',
         'GEMSTONE': 'taking a mineral sample for assay'}
# underground (site = 'mine'): the same actions, said the way a mine rescue team would
TASKS_MINE = dict(TASKS, RADIATION='ventilating the radon pocket (fan + duct)',
                  GAS='ventilating the bad air (fan + duct)', STRUCTURAL='setting a roof prop',
                  VICTIM='delivering oxygen and first aid')
VISION_KIND = {'radiation': 'RADIATION', 'fire': 'THERMAL', 'thermal': 'THERMAL', 'gas': 'GAS'}


@dataclass
class ExecParams(Params):
    coverage: bool = False              # no exploration: the beacons say where to go
    unknown_is_free: bool = True
    unknown_cost: float = 1.5
    home_on_finish: bool = False
    initial_spin: bool = False          # it plans through unknown space: no need to map first
    target_kinds: str = 'RADIATION,THERMAL,GAS,VICTIM'
    target_ids: str = ''                # e.g. "12,30": only these beacons
    min_confidence: float = 0.15        # skip events that have faded below this
    age_mode: str = 'mission'           # 'mission': enter right after the Writer; 'wallclock'
    standoff: float = 1.6               # treat hazards from this distance (m)
    victim_standoff: float = 0.8
    treat_time: float = 4.0             # s (the demo "treatment": -p treat_time:=6 for longer)
    waypoint_switch: float = 0.9        # head for the next beacon when this close to the current one
    settle_time: float = 2.0            # s without new records before the mission starts
    link_wait: float = 20.0             # s to wait for a missing beacon of a hazard's chain to the exit
    max_leg_fail: int = 3
    treat_max: float = 3.0              # m: a hazard in plain sight this close can be treated from where
                                        # the robot is (the camera recognises blocks up to ~3.2 m)
    approach_tries: int = 4             # standoff points tried before a hazard is skipped
    approach_max_path: float = 12.0     # m: a longer way to a standoff point is a search, not an approach
    approach_timeout: float = 60.0      # s per standoff point
    refine_range: float = 2.6           # m: only closer camera sightings move a hazard (the range of a
                                        # far block measured from its floor contact can be 20 % off)
    retry_skipped: bool = True          # before going home, one more attempt at every skipped hazard
    wait_briefing: float = 0.0          # s to wait for a Command Post briefing once the records are in
                                        # (0: go for every hazard straight away; a briefing still re-targets)
    site: str = ''                      # 'mine': mine wording of the tasks (resources are sampled when briefed)
    priority_kinds: str = ''            # e.g. "VICTIM": these first (nearest of them), then the others
                                        # (run_executor.sh mine: victims first). A briefing's order wins.


@dataclass
class Node:
    id: int
    kind: str
    x: float                    # record position (the event for hazards)
    y: float
    parent: Optional[int]
    next_dist: float
    next_bearing: Optional[float]
    rec: dict
    drop: Optional[Tuple[float, float]] = None     # where the beacon lies
    depth: float = 0.0          # distance along the tree to the root
    haz: Optional[Hazard] = None                   # its keep-out zone
    cam_n: int = 0              # close sightings by the Executor's own camera (they refine x, y)
    seen_t: float = -1e9        # last time the Executor's camera saw it (any range)


class ExecutorCore(ExplorerCore):
    LOG_TAG = 'executor'

    def __init__(self, params: Optional[ExecParams] = None, log=print, geo=None, wall_clock=None):
        super().__init__(params or ExecParams(), log=log)
        self.geo = geo                  # mission_log.GeoFrame (map yaw for next vectors)
        self.wall_clock = wall_clock
        self.records: Dict[int, dict] = {}
        self.records_t = None           # time the last NEW record arrived
        self.early_sightings = []       # camera sightings before the tree exists
        self.nodes: Dict[int, Node] = {}
        self.root = None
        self.targets: List[int] = []
        self.done_targets: List[dict] = []
        self.skipped: List[Tuple[int, str]] = []
        self.current = None             # node id the robot is at (tree position)
        self.target = None
        self.phase = 'WAIT'             # WAIT GOTO APPROACH TREAT RETURN DONE
        self.route: List[Tuple[float, float, Optional[int]]] = []
        self.route_i = 0
        self.need_plan = False
        self.leg_fail = 0
        self.t_mission = None
        self.ref_ts = 0
        self.treat_t = None
        self.confirmed = set()
        self.mission_events: List[dict] = []
        self.leg_recover = 0
        self.ended_away = False
        self.plan_fail = None
        self.approach_ref = None        # hazard position the approach was planned for
        self.approach_pick = None       # standoff point being driven to
        self.approach_bad = []          # standoff points that failed (this target)
        self.approach_tries = 0
        self.approach_t0 = 0.0
        self.retried = set()            # skipped targets already given a second attempt
        self.briefing = None            # v9: {'mission_id', 'seq', 'targets', 'return_to_exit', 'abort', 'in_order'}
        self.briefing_t = None
        self.brief_wait_t0 = None
        self.status = 'waiting for beacon records'

    # ================================================================ inputs
    def add_record(self, rec: dict, now: float):
        """A decoded LMB2 record (lmb2.record_to_dict + map 'x', 'y')."""
        bid = int(rec['beacon_id'])
        old = self.records.get(bid)
        if old is not None and int(old.get('seq', 0)) >= int(rec.get('seq', 0)):
            return False
        self.records[bid] = rec
        self.records_t = now
        if self.phase not in ('WAIT',):
            self._build_tree()
            n = self.nodes.get(bid)
            if n and n.kind in KEEPOUT_KINDS and not rec.get('retracted'):
                self._keepout(n, now)
            # a record that arrived late may be (or connect) a new target
            # (v9: with a briefing, only a briefed beacon whose record or chain to the exit was missing)
            for t in self._select_targets(now, log=False):
                if t not in self.targets:
                    self.targets.append(t)
                    self.skipped = [s_ for s_ in self.skipped if s_[0] != t]
                    self._event(now, f'new target from a late record: {self.nodes[t].kind} #{t}')
            if self.briefing:
                order = {t: i for i, t in enumerate(self.briefing['targets'])}
                self.targets.sort(key=lambda t: order.get(t, 1 << 20))
            if self.phase in ('RETURN', 'DONE') and (not self.ended_away or self.briefing) and self._pending():
                if self.state == 'DONE':
                    self._set_state('NAV', now)
                self.ended_away = False
                self._next_target(now)
        return True

    def on_sighting(self, kind: str, x: float, y: float, now: float):
        """The Executor's own camera saw a hazard block."""
        if not self.nodes:
            # before the mission starts there is no tree to match the sighting
            # with: keep it, it is replayed once the records are known
            self.early_sightings = (self.early_sightings + [(kind, x, y)])[-50:]
            return
        k = VISION_KIND.get(kind, kind.upper())
        near = [n for n in self.nodes.values() if n.kind == k and math.hypot(n.x - x, n.y - y) < 2.0]
        if near:
            n = min(near, key=lambda n: math.hypot(n.x - x, n.y - y))
            d = math.hypot(n.x - x, n.y - y)
            if n.id not in self.confirmed:
                self.confirmed.add(n.id)
                self._event(now, f'camera confirms {k} #{n.id} ({d:.2f} m from its record)')
            n.seen_t = now
            if self.pose is not None and math.hypot(x - self.pose[0], y - self.pose[1]) > self.p.refine_range:
                return                  # too far to measure it well: a confirmation only
            # the camera measures the hazard in the Executor's OWN map frame:
            # trust it more than the record (which is in the Writer's frame)
            n.cam_n += 1
            wgt = 0.7 if n.cam_n == 1 else 1.0 / (n.cam_n + 1)
            ox, oy = n.x, n.y
            n.x += (x - n.x) * wgt
            n.y += (y - n.y) * wgt
            if n.haz is not None:
                n.haz.x, n.haz.y = n.x, n.y
            if self.phase == 'APPROACH' and self.target == n.id and math.hypot(n.x - ox, n.y - oy) > 0.05 \
                    and self.approach_ref is not None:
                moved = math.hypot(n.x - self.approach_ref[0], n.y - self.approach_ref[1])
                if moved > 0.3:
                    self.approach_ref = (n.x, n.y)
                    self._request_plan(now, 'hazard position refined by the camera')
            return
        if k in MARKER_KINDS:
            return                      # a resource / search marker the Writer did not record: nothing to avoid
        # not in any record: a hazard the Writer did not report - keep out
        if self.phase in ('APPROACH', 'TREAT') and self.target is not None and \
                math.hypot(self.nodes[self.target].x - x, self.nodes[self.target].y - y) < 2.0:
            return                      # the target itself, seen from another side
        if self.state == 'TREAT':
            if not any(h.kind == kind and math.hypot(h.x - x, h.y - y) < self.p.hazard_merge_dist
                       for h in self.hazards):
                self.hazards.append(Hazard(kind, x, y, 1, now))
            return
        self.add_hazard(kind, x, y, now)

    # ================================================================ tree
    def _build_tree(self):
        nodes = {}
        for bid, r in self.records.items():
            nx = r.get('next') if isinstance(r.get('next'), dict) and 'id' in r['next'] else None
            nodes[bid] = Node(id=bid, kind=str(r.get('kind', 'WAYPOINT')).upper(), x=float(r['x']), y=float(r['y']),
                              parent=int(nx['id']) if nx else None,
                              next_dist=float(nx.get('dist_m') or 0.0) if nx else 0.0,
                              next_bearing=nx.get('bearing_deg') if nx else None, rec=r)
        # drop points, oldest first (a beacon always links to an older one)
        for bid in sorted(nodes):
            n = nodes[bid]
            if n.kind in ('WAYPOINT', 'EXIT'):
                n.drop = (n.x, n.y)
            p = nodes.get(n.parent) if n.parent is not None else None
            if n.parent is not None and (p is None or n.parent >= bid):
                n.parent = None                  # unknown / invalid link
                p = None
            if p is not None:
                n.depth = p.depth + n.next_dist
                if n.drop is None and p.drop is not None and n.next_bearing is not None:
                    b = math.radians(n.next_bearing)
                    e, nn = n.next_dist * math.sin(b), n.next_dist * math.cos(b)
                    dx, dy = self.geo.from_en(e, nn) if self.geo else (e, nn)
                    n.drop = (p.drop[0] - dx, p.drop[1] - dy)
        for bid, old in self.nodes.items():         # keep what the Executor learnt itself
            n = nodes.get(bid)
            if n is not None:
                n.haz, n.cam_n, n.seen_t = old.haz, old.cam_n, old.seen_t
                if old.cam_n:
                    n.x, n.y = old.x, old.y
        self.nodes = nodes
        exits = [n for n in nodes.values() if n.kind == 'EXIT' and n.parent is None]
        roots = exits or [n for n in nodes.values() if n.parent is None]
        self.root = min(roots, key=lambda n: n.id).id if roots else None

    def _missing_links(self):
        """Hazard records whose chain of `next` beacons does not reach an EXIT
        record yet (a beacon on the way has not been received)."""
        out = []
        for bid, r in self.records.items():
            if str(r.get('kind', '')).upper() in ('WAYPOINT', 'EXIT') or r.get('retracted'):
                continue
            cur, seen = bid, set()
            while cur in self.records and cur not in seen:
                seen.add(cur)
                rc = self.records[cur]
                if str(rc.get('kind', '')).upper() == 'EXIT':
                    break
                nx = rc.get('next') if isinstance(rc.get('next'), dict) else None
                cur = int(nx['id']) if nx and 'id' in nx else None
            else:
                out.append(bid)
        return sorted(out)

    def _chain(self, bid):
        """bid, its parent, ... up to its root."""
        out, seen = [], set()
        while bid is not None and bid in self.nodes and bid not in seen:
            out.append(bid)
            seen.add(bid)
            bid = self.nodes[bid].parent
        return out

    def _tree_path(self, a, b):
        """Beacon ids from a to b along the tree (up to the common ancestor,
        then down), or None if they are not connected."""
        ca, cb = self._chain(a), self._chain(b)
        sb = {v: i for i, v in enumerate(cb)}
        for i, v in enumerate(ca):
            if v in sb:
                return ca[:i + 1] + list(reversed(cb[:sb[v]]))
        return None

    def _tree_dist(self, a, b):
        path = self._tree_path(a, b)
        if path is None:
            return INF
        return sum(math.hypot(self.nodes[u].drop[0] - self.nodes[v].drop[0],
                              self.nodes[u].drop[1] - self.nodes[v].drop[1])
                   for u, v in zip(path[:-1], path[1:])
                   if self.nodes[u].drop and self.nodes[v].drop)

    # ================================================================ mission
    def _age_now(self, now):
        if self.p.age_mode == 'wallclock' and self.wall_clock is not None:
            return self.wall_clock()
        return self.ref_ts + (now - (self.t_mission or now))

    def eff_conf(self, n: Node, now) -> float:
        r = n.rec
        age = max(0.0, self._age_now(now) - float(r.get('ts', 0)))
        if age > float(r.get('ttl_s', 1e9)):
            return 0.0
        hl = max(1.0, float(r.get('half_life_s', 3600)))
        return float(r.get('confidence', 1.0)) * 2.0 ** (-age / hl)

    def _keepout(self, n: Node, now):
        if n.haz is not None:
            return
        n.haz = Hazard(n.kind.lower(), n.x, n.y, 5, now)
        self.hazards.append(n.haz)
        if self.state == 'NAV' and self._path_hits_keepout(0.0, [self.hazards[-1]]):
            self._request_plan(now, f'route crosses the zone of {n.kind} #{n.id}')

    # ================================================================ v9: briefings
    def set_briefing(self, m: dict, now: float) -> bool:
        """A verified MISSION briefing from the Command Post. Returns True if it is new."""
        try:
            mid, seq = int(m['mission_id']), int(m.get('seq', 1))
            targets = [int(t) for t in m.get('targets', [])]
        except (KeyError, TypeError, ValueError):
            return False
        b = self.briefing
        if b is not None and b['mission_id'] == mid and b['seq'] == seq:
            return False                                    # a repeat of the one we have (the ONA re-sends)
        seen, ordered = set(), []
        for t in targets:
            if t not in seen:
                seen.add(t)
                ordered.append(t)
        self.briefing = {'mission_id': mid, 'seq': seq, 'targets': ordered,
                         'return_to_exit': bool(m.get('return_to_exit', True)), 'abort': bool(m.get('abort')),
                         'in_order': bool(m.get('in_order', True))}
        self.briefing_t = now
        if self.briefing['abort']:
            self._event(now, f'BRIEFING {mid}: ABORT - back to the exit')
        else:
            self._event(now, f'BRIEFING {mid} from the Command Post: '
                        + (' -> '.join(f'#{t}' for t in ordered) or 'no targets')
                        + ('' if self.briefing['return_to_exit'] else ', then stay there'))
        if self.phase == 'WAIT':
            return True                                     # _start_mission uses it
        # during the mission: re-target now (a hazard being treated is finished first)
        self.retried = set()
        self.skipped = [s_ for s_ in self.skipped if s_[0] not in ordered]
        self.targets = self._select_targets(now)
        for bid, why in self.skipped:
            if bid in ordered:
                self._event(now, f'briefed target #{bid} skipped: {why}')
        keep = (self.phase == 'TREAT' and not self.briefing['abort']) or \
               (self.phase == 'APPROACH' and self.target is not None and self.target == (self._pending() or [None])[0])
        if not keep:
            self._resume(now)
        return True

    def _resume(self, now):
        """Pick the next target from where the robot is (after a briefing or a late record)."""
        if self.state in ('DONE', 'TREAT', 'WAIT'):
            self._set_state('NAV', now)     # (a recovery or a retreat finishes first, then plans the new route)
        self.path = []
        self.approach_pick = None
        self.ended_away = False
        self.leg_fail = 0
        self._next_target(now)

    def progress(self) -> dict:
        """For the robot's position reports (ROBOT frames) and the dashboard."""
        done_ids = {d['id'] for d in self.done_targets}
        b = self.briefing
        return {'phase': self.phase, 'mission_id': b['mission_id'] if b else 0, 'mission_ack': b is not None,
                'done': sum(1 for t in self.targets if t in done_ids), 'total': len(self.targets),
                'last_beacon': self.current, 'target': self.target}

    def _is_visit(self, bid) -> bool:
        n = self.nodes.get(bid)
        return n is not None and n.kind in ('WAYPOINT', 'EXIT')

    def _select_targets(self, now, log=True):
        kinds = [k.strip().upper() for k in self.p.target_kinds.split(',') if k.strip()]
        ids = []
        for t in self.p.target_ids.replace(' ', '').split(','):
            if t.isdigit():
                ids.append(int(t))
        briefed = self.briefing is not None
        if briefed:
            ids = [] if self.briefing['abort'] else list(self.briefing['targets'])
            if not ids:
                return []
        # "not connected" is not final: a record that arrives later may link it
        skipped = {s[0] for s in self.skipped if not s[1].startswith('not connected')}
        cands = []
        order = {t: i for i, t in enumerate(ids)}
        for n in sorted(self.nodes.values(), key=lambda n: (order.get(n.id, 1 << 20), n.id)):
            if (ids and n.id not in ids) or (not ids and n.kind not in kinds) or n.id in skipped:
                continue
            why = None
            if n.rec.get('retracted') and not briefed:
                why = 'retracted (already handled)'
            elif self.eff_conf(n, now) < self.p.min_confidence and not briefed:
                # (a briefed target is checked anyway: the operator asked for it)
                why = f'faded (confidence {self.eff_conf(n, now):.2f})'
            elif n.drop is None or self._chain(n.id)[-1] != self.root:
                why = 'not connected to the exit'
            if why is None:
                cands.append(n.id)
            elif log and n.id not in {s_[0] for s_ in self.skipped}:
                self.skipped.append((n.id, why))
        if briefed and log:
            for t in ids:
                if t not in self.nodes and t not in {s_[0] for s_ in self.skipped}:
                    self.skipped.append((t, 'not connected: no record of this beacon yet'))
        return cands

    def _pending(self):
        done = {d['id'] for d in self.done_targets} | {s[0] for s in self.skipped}
        return [t for t in self.targets if t not in done]

    def _start_mission(self, now):
        self.t_mission = now
        self.ref_ts = max(int(r.get('ts', 0)) for r in self.records.values())
        self._build_tree()
        for n in self.nodes.values():
            if n.kind in KEEPOUT_KINDS and not n.rec.get('retracted'):
                self._keepout(n, now)
        cands = self._select_targets(now)
        self.targets = cands
        self.current = self.root
        n_trail = sum(1 for n in self.nodes.values() if n.kind == 'WAYPOINT')
        self._event(now, f'{len(self.records)} beacon records ({n_trail} trail, '
                         f'{len(self.nodes) - n_trail} other), exit #{self.root}; '
                         + (f'briefing {self.briefing["mission_id"]}: ' if self.briefing else 'mission: ')
                         + f'{len(cands)} target(s) ' +
                    ', '.join(f'{self.nodes[t].kind} #{t}' for t in cands))
        for bid, why in self.skipped:
            self._event(now, f'target #{bid} skipped: {why}')
        early, self.early_sightings = self.early_sightings, []
        for kind, x, y in early:
            self.on_sighting(kind, x, y, now)
        self._next_target(now)

    def _next_target(self, now):
        left = self._pending()
        if not left and self.p.retry_skipped:
            # before going home: one more attempt at each hazard skipped on the
            # way (the robot now knows much more of the map)
            again = [b for b, _ in self.skipped if b in self.targets and b not in self.retried]
            if again:
                t = min(again, key=lambda t: self._tree_dist(self.current, t))
                why = next(w for b, w in self.skipped if b == t)
                self.retried.add(t)
                self.skipped = [s_ for s_ in self.skipped if s_[0] != t]
                self._event(now, f'second attempt at {self.nodes[t].kind} #{t} (skipped earlier: {why})')
                left = [t]
        if left:
            first = [k.strip().upper() for k in self.p.priority_kinds.split(',') if k.strip()]
            urgent = [t for t in left if self.nodes[t].kind in first]
            if self.briefing and self.briefing['in_order']:
                t = left[0]                     # the Command Post's order
            elif urgent:                        # e.g. victims before anything else
                t = min(urgent, key=lambda t: (first.index(self.nodes[t].kind), self._tree_dist(self.current, t)))
            else:
                t = min(left, key=lambda t: self._tree_dist(self.current, t))
            self.target = t
            n = self.nodes[t]
            ids = self._tree_path(self.current, t)
            self._set_route(ids, now)
            self.phase = 'GOTO'
            self._event(now, f'GOTO {n.kind} #{t}: {len(ids)} beacons, '
                             f'{self._tree_dist(self.current, t):.1f} m along the beacon chain')
        else:
            self.target = None
            if self.briefing and not self.briefing['return_to_exit'] and not self.briefing['abort']:
                self.phase = 'DONE'
                self._set_state('DONE', now)
                self.path = []
                self.route = []
                self.ended_away = True
                self.status = f'briefing {self.briefing["mission_id"]} done - waiting here for orders'
                self._event(now, f'BRIEFING {self.briefing["mission_id"]} COMPLETE: '
                                 f'{self.progress()["done"]}/{len(self.targets)} targets, waiting at beacon '
                                 f'#{self.current} for the next briefing')
                return
            ids = self._chain(self.current) if self.current is not None else []
            self._set_route(ids, now, extra=[self.home] if self.home else [])
            self.phase = 'RETURN'
            self._event(now, f'RETURN to the exit: following {len(ids)} "next" hops')

    def _set_route(self, ids, now, extra=()):
        pts = []
        for i in ids:
            n = self.nodes[i]
            if n.drop is not None:
                pts.append((n.drop[0], n.drop[1], i))
        for q in extra:
            pts.append((q[0], q[1], None))
        # merge points closer than 0.3 m
        route = []
        for q in pts:
            if route and math.hypot(q[0] - route[-1][0], q[1] - route[-1][1]) < 0.3:
                route[-1] = q
            else:
                route.append(q)
        # start from the first point not already behind us
        x, y = self.pose[:2]
        k = 0
        while k < len(route) - 1 and math.hypot(route[k][0] - x, route[k][1] - y) < self.p.waypoint_switch:
            k += 1
        self.route, self.route_i = route, k
        self.leg_fail = 0
        self.leg_recover = 0
        self._request_plan(now, 'new route')

    def _treat_max(self, n: Node):
        return self.p.victim_standoff + 0.8 if n.kind in CLOSE_KINDS else max(self.p.treat_max, self.p.standoff)

    def _standoff(self, n: Node):
        return self.p.victim_standoff if n.kind in CLOSE_KINDS else self.p.standoff

    def _task(self, n: Node):
        return (TASKS_MINE if self.p.site == 'mine' else TASKS).get(n.kind, 'treating')

    def _approach_candidates(self, n: Node):
        """Standoff points around the hazard: rings from `standoff` out to
        `treat_max`, points ~0.3 m apart, each with a clear view of the hazard on the
        robot's own map. Extra cost (m): farther rings and directions away
        from where the Writer saw it from are less preferred, so the robot
        takes the Writer's line of sight when it can, any other side when it
        cannot, and a farther ring only if no nearer one is reachable (or it
        saves a detour of several metres)."""
        p = self.p
        sd = self._standoff(n)
        rmax = self._treat_max(n)
        rings = [r for r in (sd, sd + 0.4, sd + 0.8, sd + 1.2) if r <= max(sd, rmax - 0.2) + 1e-6]
        ref = n.drop if n.drop is not None else self.pose[:2]
        a0 = math.atan2(ref[1] - n.y, ref[0] - n.x)
        g = self.grid
        others = [h for h in self.hazards if h is not n.haz and math.hypot(h.x - n.x, h.y - n.y) > 0.3]
        out, blind = [], []
        for r in rings:
            m = max(18, int(math.ceil(2 * math.pi * r / 0.3)))     # points ~0.3 m apart on the ring
            for k in range(m):
                da = wrap(k * 2 * math.pi / m)
                x, y = n.x + r * math.cos(a0 + da), n.y + r * math.sin(a0 + da)
                if any(math.hypot(x - bx, y - by) < 0.5 for bx, by in self.approach_bad):
                    continue
                if any(math.hypot(x - h.x, y - h.y) < p.keepout_radius + 0.1 for h in others):
                    continue
                cost = 6.0 * (r - sd) + 1.0 * abs(da) / math.pi
                if g is not None:
                    k_stop = (r - 0.4) / r                  # the block itself is an obstacle
                    a, b = g.cell(x, y), g.cell(x + (n.x - x) * k_stop, y + (n.y - y) * k_stop)
                    if not g.inside(*a):
                        continue
                    if not self._line_free(g, a, b):
                        blind.append((x, y, cost + 3.0))
                        continue
                    # a view through space the map has not seen yet may still
                    # hit a wall: prefer views through known free space
                    cost += 3.0 * self._unknown_share(g, a, b)
                out.append((x, y, cost))
        # no clear view from anywhere: the hazard position itself is probably
        # off (inside a wall on this map). Try the points anyway; on arrival the
        # camera decides.
        return out or blind

    @staticmethod
    def _unknown_share(g, a, b):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        cnt = 0
        for t in np.linspace(0, 1, n + 1):
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            cnt += bool(g.inside(i, j) and g.unknown[i, j])
        return cnt / (n + 1)

    def _plan_approach(self, now):
        """Plan to the cheapest reachable standoff point of the target."""
        n = self.nodes[self.target]
        self._build_grid()
        cands = self._approach_candidates(n)
        if not cands:
            self.plan_fail = 'target'
            return False
        if not self._plan(now, targets=cands):
            return False
        pts = [self.pose[:2]] + list(self.path)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
        if length > self.p.approach_max_path:
            # only a long detour through space nobody has seen: no real way in
            self.path = []
            self.plan_fail = 'too far'
            return False
        self.approach_pick = self.plan_pick
        self.route = [(self.goal[0], self.goal[1], None)]
        self.route_i = 0
        if self.p.verbose:
            d = math.hypot(self.plan_pick[0] - n.x, self.plan_pick[1] - n.y)
            self.log(f'[executor] standoff point ({self.plan_pick[0]:.2f}, {self.plan_pick[1]:.2f}), '
                     f'{d:.1f} m from {n.kind} #{n.id}, {len(cands)} candidates')
        return True

    def _leg_done(self, now):
        if self.phase == 'GOTO' and self._is_visit(self.target):
            # a trail / exit beacon in the briefing: being there is the task
            n = self.nodes[self.target]
            self.current = n.id
            ev = {'id': n.id, 'kind': n.kind, 'x': n.x, 'y': n.y, 'distance': 0.0, 'confirmed_by_camera': False,
                  't': now, 'task': 'visited'}
            self.done_targets.append(ev)
            self.mission_events.append(ev)
            self._event(now, f'{n.kind} #{n.id} VISITED (briefing)')
            self.leg_fail = 0
            self._next_target(now)
            return
        if self.phase == 'GOTO':
            n = self.nodes[self.target]
            self.current = n.id
            self.phase = 'APPROACH'
            self.approach_ref = (n.x, n.y)
            self.approach_pick = None
            self.approach_bad = []
            self.approach_tries = 0
            self.approach_t0 = now
            self.route = []
            self.route_i = 0
            self.leg_fail = 0
            self._event(now, f'at beacon #{n.id}: approaching the {n.kind.lower()} to '
                             f'{self._standoff(n):.1f} m')
            self._request_plan(now, 'approach')
        elif self.phase == 'APPROACH':
            n = self.nodes[self.target]
            if not self._in_sight(n, now):
                # the standoff point was chosen through space the map had not
                # seen yet: a wall stands between the robot and the hazard
                self._leg_failed(now, 'no clear view of it from the standoff point')
                return
            self.phase = 'TREAT'
            self.treat_t = None
            self._set_state('TREAT', now)
            self._event(now, f'{n.kind} #{n.id} reached ({math.hypot(self.pose[0] - n.x, self.pose[1] - n.y):.2f} m):'
                             f' {self._task(n)}')
        elif self.phase == 'TREAT':
            self._set_state('TREAT', now)
        elif self.phase == 'RETURN':
            self.phase = 'DONE'
            self._set_state('DONE', now)
            self.path = []
            missing = self._missing_links()
            self._event(now, f'MISSION COMPLETE: {self.progress()["done"]}/{len(self.targets)} targets treated, '
                             'back at the exit' + (' (waiting for the beacons linking '
                                                   + ', '.join(f'#{b}' for b in missing) + ')' if missing else ''))

    def _stops_at_goal(self):
        # an intermediate beacon of the route: the next leg is planned 0.9 m
        # before it (waypoint_switch), so do not slow down for it
        return self.state != 'NAV' or self.route_i >= len(self.route) - 1

    def _in_sight(self, n: Node, now):
        """The hazard is in sight: no wall on the map between the robot and it,
        or the camera saw it in the last 2 s (its map position may be off)."""
        return now - n.seen_t < 2.0 or self._clear_view(n.x, n.y)

    def _clear_view(self, hx, hy):
        """No wall between the robot and the hazard (the block itself excluded)."""
        if self.grid is None or self.pose is None:
            return False
        rx, ry = self.pose[:2]
        d = math.hypot(hx - rx, hy - ry)
        if d < 0.45:
            return True
        k = (d - 0.4) / d                       # stop 0.4 m short of the block's centre
        g = self.grid
        return self._line_free(g, g.cell(rx, ry), g.cell(rx + (hx - rx) * k, ry + (hy - ry) * k))

    def _leg_failed(self, now, why):
        if self.phase == 'APPROACH':
            n = self.nodes[self.target]
            d = math.hypot(self.pose[0] - n.x, self.pose[1] - n.y)
            if d <= self._treat_max(n) + 0.2 and self._in_sight(n, now):
                # close enough for the tool, in plain sight: treat from here
                self._event(now, f'{n.kind} #{n.id}: approach failed ({why}), but it is in plain sight '
                                 f'{d:.1f} m away - treating from here')
                self.route = []
                self.path = []
                self._leg_done(now)
                return
            if self.approach_pick is not None:
                self.approach_bad.append(self.approach_pick)
                self.approach_pick = None
            self.approach_tries += 1
            if self.approach_tries < self.p.approach_tries:
                self._event(now, f'{n.kind} #{n.id}: approach failed ({why}) - trying another standoff point '
                                 f'({self.approach_tries + 1}/{self.p.approach_tries})')
                self.leg_fail = 0
                self.leg_recover = 0
                self.route = []
                self.approach_t0 = now
                self._request_plan(now, 'approach from another side')
                return
        if self.phase in ('GOTO', 'APPROACH'):
            self._event(now, f'target #{self.target} unreachable ({why}) - skipped')
            self.skipped.append((self.target, why))
            self._next_target(now)
            return
        # RETURN: go straight for the exit / start
        if self.route_i < len(self.route) - 1:
            self.route_i += 1
            self._request_plan(now, 'next hop')
        else:
            self.phase = 'DONE'
            self.ended_away = True
            self.status = f'MISSION ENDED away from the exit ({why})'
            self._set_state('DONE', now)
            self._event(now, f'MISSION ENDED away from the exit ({why})')

    # ================================================================ low level
    def _request_plan(self, now, why):
        if self.p.verbose:
            self.log(f'[executor] replan: {why}')
        self.path = []
        self.need_plan = True
        if self.state not in ('DONE', 'TREAT'):
            self._set_state('NAV', now)

    def _build_grid(self, with_scan=True):
        # pad the SLAM map with unknown space so that every point of the route
        # (possibly not seen yet) lies inside the planning grid
        data, res, ox, oy = self.map
        pts = [(q[0], q[1]) for q in self.route]
        if self.pose is not None:
            pts.append(self.pose[:2])
        if self.phase == 'APPROACH' and self.target in self.nodes:
            n = self.nodes[self.target]
            r = self._treat_max(n)
            pts += [(n.x - r, n.y - r), (n.x + r, n.y + r)]
        if pts:
            h, w = data.shape
            xs = [q[0] for q in pts]
            ys = [q[1] for q in pts]
            m = 1.5
            nx0 = min(ox, min(xs) - m)
            ny0 = min(oy, min(ys) - m)
            nx1 = max(ox + w * res, max(xs) + m)
            ny1 = max(oy + h * res, max(ys) + m)
            j0 = int(math.ceil((ox - nx0) / res))
            i0 = int(math.ceil((oy - ny0) / res))
            W = j0 + w + int(math.ceil((nx1 - (ox + w * res)) / res))
            H = i0 + h + int(math.ceil((ny1 - (oy + h * res)) / res))
            if (W, H) != (w, h):
                big = np.full((H, W), -1, dtype=data.dtype)
                big[i0:i0 + h, j0:j0 + w] = data
                self.map = (big, res, ox - j0 * res, oy - i0 * res)
        g = super()._build_grid(with_scan)
        self.map = (data, res, ox, oy)
        return g

    def _plan_leg(self, now):
        """Plan to the current route point; skip route points that cannot be
        reached. Returns True if a path is set."""
        if self.phase == 'APPROACH' and self.target is not None:
            if self._plan_approach(now):
                self.need_plan = False
                return True
            return False
        while self.route_i < len(self.route):
            tx, ty, bid = self.route[self.route_i]
            if self._plan(now, target=(tx, ty)):
                self.need_plan = False
                return True
            if self.plan_fail == 'start':
                return False            # the robot is boxed in, not the beacon: recover first
            if self.route_i < len(self.route) - 1:
                if self.p.verbose:
                    self.log(f'[executor] beacon #{bid} not reachable - next one')
                self.route_i += 1
                continue
            return False
        return False

    # ================================================================ step
    def step(self, now: float):
        p = self.p
        if self.map is None or self.pose is None or self.scan is None:
            self.status = 'waiting for map, pose and scan'
            return 0.0, 0.0
        self._tick(now)
        if self.phase == 'WAIT':
            if not self.records or self.records_t is None or now - self.records_t < p.settle_time:
                self.status = f'receiving beacon records ({len(self.records)})'
                return 0.0, 0.0
            has_exit = any(str(r.get('kind', '')).upper() == 'EXIT' for r in self.records.values())
            if not has_exit and now - self.records_t < 4 * p.settle_time:
                self.status = f'receiving beacon records ({len(self.records)}), waiting for the EXIT beacon'
                return 0.0, 0.0
            missing = self._missing_links()
            if missing and now - self.records_t < p.link_wait:
                # a beacon between a hazard and the exit has not been heard yet:
                # the radio repeats every record, give it time
                self.status = (f'receiving beacon records ({len(self.records)}), waiting for the link of '
                               + ', '.join(f'#{b}' for b in missing[:4]) + ' to the exit')
                return 0.0, 0.0
            if self.briefing is None and p.wait_briefing > 0:
                if self.brief_wait_t0 is None:
                    self.brief_wait_t0 = now
                    self._event(now, f'{len(self.records)} beacon records in: waiting up to {p.wait_briefing:.0f} s '
                                     'for a briefing from the Command Post')
                if now - self.brief_wait_t0 < p.wait_briefing:
                    self.status = (f'{len(self.records)} beacon records - waiting for a briefing from the Command '
                                   f'Post ({p.wait_briefing - (now - self.brief_wait_t0):.0f} s left)')
                    return 0.0, 0.0
                self._event(now, 'no briefing heard: going for every live hazard')
            if self.state == 'WAIT':
                self._set_state('NAV', now)
            self._start_mission(now)
            return 0.0, 0.0

        if self.map_stamp != self.grid_stamp:
            self._build_grid()
            if self.state == 'NAV' and self.path and self._path_blocked():
                self._request_plan(now, 'path blocked on the updated map')

        if self.state == 'DONE':
            if self.phase == 'DONE' and not self.ended_away:
                self.status = 'MISSION COMPLETE - stopped at the exit'
            return 0.0, 0.0
        if self.state == 'RECOVER':
            return self._recover(now)
        if self.state == 'TREAT':
            return self._treat(now)
        if self.state == 'RETREAT':
            v, w, arrived = self._follow(now, reverse_first=True)
            if arrived or now - self.t_state > 45.0:
                self._event(now, 'retreat finished - back on the beacon route')
                self._request_plan(now, 'after retreat')
                return 0.0, 0.0
            return self._guard(now, v, w)

        # NAV
        if self.need_plan or not self.path:
            if now - self.t_state < 0.1:
                return 0.0, 0.0
            if not self._plan_leg(now):
                self.leg_fail += 1
                if self.leg_fail >= p.max_leg_fail or (self.phase == 'APPROACH' and self.plan_fail != 'start'):
                    self._leg_failed(now, 'only a long detour through unmapped space'
                                     if self.plan_fail == 'too far' else 'no path')
                else:
                    self._start_recover(now, 'no path to the next beacon' if self.plan_fail != 'start'
                                        else 'no free space around the robot')
                return 0.0, 0.0
        if self.phase == 'APPROACH' and now - self.approach_t0 > p.approach_timeout:
            self._leg_failed(now, f'no standoff point reached in {p.approach_timeout:.0f} s')
            return 0.0, 0.0
        x, y = self.pose[:2]
        # close to the current beacon: head for the next one without stopping
        if self.route_i < len(self.route) - 1:
            tx, ty, bid = self.route[self.route_i]
            if math.hypot(tx - x, ty - y) < p.waypoint_switch:
                if bid is not None:
                    self.current = bid          # we are at this beacon of the tree
                self.route_i += 1
                self.leg_fail = 0
                self.leg_recover = 0
                if not self._plan_leg(now):
                    self._request_plan(now, 'next beacon')
                    return 0.0, 0.0
        v, w, arrived = self._follow(now)
        if arrived:
            bid = self.route[self.route_i][2] if self.route else None
            if bid is not None:
                self.current = bid
            if self.route_i < len(self.route) - 1:
                self.route_i += 1
                self.leg_fail = 0
                self.leg_recover = 0
                self._request_plan(now, 'next beacon')
            else:
                self.path = []
                self._leg_done(now)
            return 0.0, 0.0
        tgt = f'{self.nodes[self.target].kind} #{self.target}' if self.target is not None else 'exit'
        rid = self.route[self.route_i][2] if self.route else None
        self.status = f'{self.phase} {tgt}: ' + (f'beacon #{rid} ' if rid is not None else '') + \
                      f'({self.route_i + 1}/{len(self.route)})'
        return self._guard(now, v, w)

    def _guard(self, now, v, w):
        """LiDAR emergency stop + progress watchdog (as the Writer)."""
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
        if now - self.progress_t > self.p.stuck_time:
            self._start_recover(now, 'no progress')
            return 0.0, 0.0
        if self._blocked(now, v, w):
            self._start_recover(now, 'blocked (told to move, not moving)')
            return 0.0, 0.0
        return v, w

    def _start_recover(self, now, why):
        """Recovery (back up, turn, replan) - but not forever on one leg."""
        self.leg_recover += 1
        if self.leg_recover > 5 and self.phase in ('GOTO', 'APPROACH', 'RETURN'):
            self.leg_recover = 0
            self._leg_failed(now, f'stuck ({why})')
            return
        super()._start_recover(now, why)

    def _treat(self, now):
        n = self.nodes[self.target]
        err = wrap(math.atan2(n.y - self.pose[1], n.x - self.pose[0]) - self.pose[2])
        if self.treat_t is None:
            if abs(err) > 0.1 and now - self.t_state < 8.0:
                self.status = f'facing {n.kind} #{n.id}'
                v, w = self._turn_cmd(math.copysign(max(0.4, min(self.p.look_w_max, 2.0 * abs(err))), err))
                return v, (self._smooth_w(w, 0.4) if w else 0.0)
            self.treat_t = now
        if now - self.treat_t < self.p.treat_time:
            self.status = f'{self._task(n)} (#{n.id}) {now - self.treat_t:.0f}/{self.p.treat_time:.0f} s'
            return 0.0, 0.0
        d = math.hypot(self.pose[0] - n.x, self.pose[1] - n.y)
        ev = {'id': n.id, 'kind': n.kind, 'x': n.x, 'y': n.y, 'distance': round(d, 2),
              'confirmed_by_camera': n.id in self.confirmed, 't': now,
              'task': self._task(n)}
        self.done_targets.append(ev)
        self.mission_events.append(ev)
        self._event(now, f'{n.kind} #{n.id} TREATED ({ev["task"]}, from {d:.2f} m'
                         + (', confirmed by camera)' if ev['confirmed_by_camera'] else ')'))
        self._set_state('NAV', now)
        self._next_target(now)
        return 0.0, 0.0
