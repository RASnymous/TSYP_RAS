"""
Zone simulator: what the three ONA gateways hear during a mission.

No ROS, no Gazebo, no radio hardware. It plays a mission from the ground up:
the beacons' records reaching each gateway through the gossip network, a robot
(the Executor, or the Writer) driving through the building and pinging every 2 s,
and each gateway measuring its distance to the robot (two-way ranging), with the
walls in the way making ranges too long, as they do on a real site.

Its output is exactly what real gateways print (one JSON line per frame, with the
frame bytes), sent over UDP to the ONA (python -m ona --udp ...). It also listens
for the ONA's briefing frames: the simulated Executor waits at the exit, receives
the signed MISSION, acknowledges it, and drives to those targets in that order.

    python -m ona.zonesim                              built-in building, Executor mission
    python -m ona.zonesim --mission demo_big.json --world contaminated_zone.world
    python -m ona.zonesim --liar gw3 --forge gw2@60 --kill 6@120 --slam-slip 90:2.0,-1.0

Faults you can inject (to show the ONA catching them):
    --liar GW          GW re-signs hazard records with a lower severity (it has the key)
    --forge GW@T       GW invents a VICTIM record at time T that no beacon ever sent
    --corrupt GW@T     from T, GW passes on frames with a flipped bit (bad signature)
    --gw-down GW@A-B   GW hears nothing between A and B seconds
    --kill ID@T        beacon ID is crushed at T: neighbours start reporting it silent
    --slam-slip T:DX,DY  the robot's own SLAM jumps by (DX, DY) m at time T
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import random
import socket
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from . import lmb2
from .core import OnaConfig

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------- geometry
def _seg_intersects_rect(p, q, rect) -> bool:
    """Segment p-q against an oriented rectangle (cx, cy, sx, sy, yaw)."""
    cx, cy, sx, sy, yaw = rect
    c, s = math.cos(-yaw), math.sin(-yaw)

    def loc(pt):
        dx, dy = pt[0] - cx, pt[1] - cy
        return dx * c - dy * s, dx * s + dy * c
    (x0, y0), (x1, y1) = loc(p), loc(q)
    hx, hy = sx / 2, sy / 2
    t0, t1 = 0.0, 1.0
    dx, dy = x1 - x0, y1 - y0
    for pp, qq in ((-dx, x0 + hx), (dx, hx - x0), (-dy, y0 + hy), (dy, hy - y0)):
        if abs(pp) < 1e-12:
            if qq < 0:
                return False
        else:
            r = qq / pp
            if pp < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return False
    return True


def load_world_boxes(path: str) -> list:
    """Static boxes of an SDF world (walls, rubble) as (cx, cy, sx, sy, yaw)."""
    boxes = []
    root = ET.parse(path).getroot()
    for m in root.iter('model'):
        name = m.get('name', '')
        if name.startswith(('beacon', 'ground', 'writer', 'executor')):
            continue
        pose = (m.findtext('pose') or '0 0 0 0 0 0').split()
        pose = [float(v) for v in pose] + [0.0] * (6 - len(pose))
        size = None
        for b in m.iter('box'):
            size = [float(v) for v in (b.findtext('size') or '').split()]
            break
        if not size or len(size) < 2:
            continue
        lp = None
        for link in m.iter('link'):
            lp = link.findtext('pose')
            break
        if lp:
            v = [float(x) for x in lp.split()] + [0.0] * 6
            pose[0] += v[0]
            pose[1] += v[1]
            pose[5] += v[5]
        boxes.append((pose[0], pose[1], size[0], size[1], pose[5]))
    return boxes


def walls_crossed(p, q, boxes) -> int:
    return sum(1 for b in boxes if _seg_intersects_rect(p, q, b))


def walls_on_path(p, q, boxes) -> list:
    return [i for i, b in enumerate(boxes) if _seg_intersects_rect(p, q, b)]


# ---------------------------------------------------------------------- built-in building
def builtin_scenario() -> dict:
    """A 22 x 14 m building: corridor along y = 0, four rooms with doors, entrance at the west."""
    W = 0.2
    boxes = [(10, 7, 22.4, W, 0), (10, -7, 22.4, W, 0), (21.1, 0, W, 14, 0),
             (-1.1, 4.0, W, 6.2, 0), (-1.1, -4.0, W, 6.2, 0)]                 # outer walls, entrance gap |y| < 0.9
    for y in (1.6, -1.6):                                                     # corridor walls with doors
        segs = [(0.0, 3.5), (5.0, 8.0), (9.5, 14.0), (15.5, 21.0)]
        for a, b in segs:
            boxes.append(((a + b) / 2, y, b - a, W, 0))
    boxes += [(11.0, 4.3, W, 5.4, 0), (11.0, -4.3, W, 5.4, 0)]                  # room dividers
    beacons = [
        (0, 'exit', (0.0, 0.0), None, None), (1, 'waypoint', (2.5, 0.0), None, 0), (2, 'waypoint', (4.3, 0.0), None, 1),
        (3, 'waypoint', (4.3, 2.6), None, 2), (4, 'radiation', (4.3, 3.6), (6.2, 5.0), 3),
        (5, 'waypoint', (7.0, 0.0), None, 2), (6, 'waypoint', (8.8, 0.0), None, 5),
        (7, 'waypoint', (8.8, -2.6), None, 6), (8, 'victim', (8.8, -3.8), (6.8, -5.2), 7),
        (9, 'waypoint', (11.5, 0.0), None, 6), (10, 'waypoint', (14.8, 0.0), None, 9),
        (11, 'waypoint', (14.8, 2.6), None, 10), (12, 'fire', (14.8, 3.8), (17.5, 5.0), 11),
        (13, 'waypoint', (14.8, -2.6), None, 10), (14, 'gas', (14.8, -3.8), (18.0, -5.0), 13),
        (15, 'waypoint', (18.0, 0.0), None, 10),
    ]
    return {'boxes': boxes, 'beacons': beacons, 'home': (0.0, 0.0), 'name': 'built-in building 22 x 14 m'}


TARGET_KINDS = ('VICTIM', 'GAS', 'RADIATION', 'THERMAL', 'STRUCTURAL')
KIND_OF = {'exit': 'EXIT', 'waypoint': 'WAYPOINT', 'radiation': 'RADIATION', 'fire': 'THERMAL',
           'thermal': 'THERMAL', 'gas': 'GAS', 'victim': 'VICTIM'}


@dataclass
class SimBeacon:
    id: int
    kind: str
    drop: np.ndarray
    event: np.ndarray
    link: Optional[int]
    frame: bytes = b''
    record: dict = field(default_factory=dict)
    alive: bool = True
    depth: int = 0


class ZoneSim:
    def __init__(self, cfg: OnaConfig, scenario: dict, seed: int = 1, speed: float = 1.0,
                 sink: Optional[Callable[[dict], None]] = None, robot: str = 'executor', wait_briefing: bool = True,
                 t0: Optional[float] = None, faults: Optional[dict] = None, ping_s: float = 2.0,
                 range_sigma: float = 0.35, wall_bias: tuple = (0.2, 1.0), v: float = 0.45, log=print):
        self.cfg = cfg
        self.key, self.net = cfg.key, cfg.net_id
        self.cal = cfg.calibrator
        self.rng = random.Random(seed)
        self.nrng = np.random.default_rng(seed)
        self.speed = speed
        self.advertise_speed = False
        self.sink = sink or (lambda d: None)
        self.log = log or (lambda s: None)
        self.boxes = scenario.get('boxes', [])
        wr = random.Random(seed + 1000)
        self.wall_props = [(wr.uniform(4.0, 9.0), wr.uniform(*wall_bias)) for _ in self.boxes]
        self.home = np.array(scenario.get('home', (0.0, 0.0)), dtype=float)
        self.faults = faults or {}
        self.ping_s = ping_s
        self.range_sigma = range_sigma
        self.wall_bias = wall_bias
        self.v = v
        self.t0 = time.time() if t0 is None else t0
        self.t = 0.0
        self.gws = {g: self.cal.enu_to_local(e) for g, e in cfg.gateways.items()}   # gateway antenna, map frame
        for g in self.gws:
            self.gws[g][2] = cfg.gateways[g][2] - self.cal.t[2]
        self.beacons: dict = {}
        self._build_beacons(scenario)
        self.events = []                  # heap (t, seq, fn)
        self._n = 0
        self.stats = {g: {'rx': 0, 'known': set()} for g in self.gws}
        # robot
        self.robot_kind = robot
        self.robot_id = 1 if robot == 'executor' else 2
        self.pos = self.home.copy()
        self.yaw = 0.0
        self.odo = 0.0
        self.slam_err = np.zeros(2)
        self.route = []
        self.phase = 'WAIT'
        self.targets = []
        self.done = 0
        self.total = 0
        self.mission_id = 0
        self.mission_ack = False
        self.treat_until = None
        self.wait_briefing = wait_briefing
        self.briefing_deadline = 25.0
        self.seq = 0
        self.asm = lmb2.MissionAssembler()
        self.finished = False
        self.last_beacon = min(self.beacons) if self.beacons else 0
        self.treated = []
        self.plan_set = None          # targets of the current plan (progress counts only these)
        self.route_ids = []
        self.current_target = None
        self.truth = {}                   # ping seq -> true (x, y), for tests

    # ------------------------------------------------------------------ setup
    def _build_beacons(self, sc: dict) -> None:
        now = int(self.t0)
        if 'records' in sc:      # a living-map-mission/1 file
            # re-time the mission so that it looks like the Writer just finished (newest record 2 min old),
            # re-signed with the mission key: a gas record from yesterday would have faded away
            recs = []
            for b in sc['records']:
                fr = bytes.fromhex(b['frame']) if b.get('frame') else lmb2.encode_record(b['record'], self.key,
                                                                                            self.net)
                recs.append((b, lmb2.decode_record(fr, self.key)))
            shift = (now - 120) - max(r['ts'] for _, r in recs) if recs else 0
            for b, rec in recs:
                rec = dict(rec, ts=rec['ts'] + shift)
                rec.pop('_hdr', None)
                fr = lmb2.encode_record(rec, self.key, self.net)
                rec = lmb2.decode_record(fr, self.key)
                kind = rec['kind']
                drop = np.array(b.get('drop', [0, 0])[:2], dtype=float)
                ev = np.array(b.get('event', b.get('drop', [0, 0]))[:2], dtype=float)
                link = (rec.get('next') or {}).get('id')
                self.beacons[rec['beacon_id']] = SimBeacon(rec['beacon_id'], kind, drop, ev, link, fr, rec)
        else:
            for bid, et, drop, event, link in sc['beacons']:
                kind = KIND_OF[et]
                drop = np.array(drop, dtype=float)
                ev = np.array(event if event is not None else drop, dtype=float)
                pos = self.cal.local_to_gps(np.array([ev[0], ev[1], 0.0]))
                nxt = None
                if link is not None:
                    parent = self.beacons[link]
                    d = parent.drop - drop
                    brg = (90.0 - math.degrees(math.atan2(*(self.cal.R[:2, :2] @ d)[::-1]))) % 360.0
                    nxt = {'id': link, 'dist_m': round(float(np.linalg.norm(d)), 1), 'bearing_deg': brg}
                hl, ttl, _ = lmb2.KIND_TABLE[kind]
                rec = {'beacon_id': bid, 'seq': 1, 'kind': kind,
                       'gps': {'lat': pos['lat'], 'lon': pos['lon'], 'err_m': round(0.3 + 0.02 * bid * 2, 1)},
                       'severity': 0 if kind in ('WAYPOINT', 'EXIT') else self.rng.randrange(25, 70),
                       'confidence': 1.0 if kind in ('WAYPOINT', 'EXIT') else 0.9, 'next': nxt,
                       'ts': now - 600 + bid * 20, 'ttl_s': ttl, 'half_life_s': hl}
                fr = lmb2.encode_record(rec, self.key, self.net)
                self.beacons[bid] = SimBeacon(bid, kind, drop, ev, link, fr, lmb2.decode_record(fr, self.key))
        for b in self.beacons.values():
            d, cur, seen = 0, b, set()
            while cur.link is not None and cur.link in self.beacons and cur.id not in seen:
                seen.add(cur.id)
                cur = self.beacons[cur.link]
                d += 1
            b.depth = d

    # ------------------------------------------------------------------ scheduling
    def at(self, t: float, fn) -> None:
        self._n += 1
        heapq.heappush(self.events, (t, self._n, fn))

    def now_unix(self) -> float:
        return self.t0 + self.t

    def emit(self, d: dict) -> None:
        d.setdefault('t', round(self.now_unix(), 3))
        self.sink(d)

    def gw_down(self, gw: str) -> bool:
        for a, b in self.faults.get('down', {}).get(gw, []):
            if a <= self.t < b:
                return True
        return False

    # ------------------------------------------------------------------ beacon traffic
    def _deliver(self, gw: str, b: SimBeacon, silent: bool = False) -> None:
        if self.gw_down(gw):
            return
        fr = bytearray(b.frame)
        relay_candidates = [x for x in self.beacons.values() if x.depth <= 2 and x.id != b.id]
        if (b.depth <= 1 and not silent) or not relay_candidates:
            relay, hops = b.id, 0                                   # heard straight from the beacon
        else:
            relay, hops = self.rng.choice(relay_candidates).id, max(1, b.depth - 1)
        fr[2:4] = int(relay).to_bytes(2, 'little')
        fr[4] = (min(15, hops) << 4) | 15
        fr[5] = 0x01 if silent else 0x00
        if self.faults.get('liar') == gw and b.kind not in ('WAYPOINT', 'EXIT'):
            rec = dict(b.record)
            rec['severity'] = 1
            rec['gps'] = dict(rec['gps'])
            rec['gps']['lat'] += 3e-5                                   # moves the hazard ~3 m
            fr = bytearray(lmb2.encode_record(rec, self.key, self.net, relay=relay))
        cor = self.faults.get('corrupt', {})
        if gw in cor and self.t >= cor[gw] and self.rng.random() < 0.5:
            fr[20] ^= 0x04
        self.stats[gw]['rx'] += 1
        self.stats[gw]['known'].add(b.id)
        rssi = -85 - 3 * b.depth + self.rng.gauss(0, 3)
        self.emit({'gw': gw, 'ev': 'rx', 'frame': bytes(fr).hex(), 'rssi': round(rssi, 1),
                   'snr': round(max(-15.0, 8 - 1.5 * b.depth + self.rng.gauss(0, 1)), 1)})

    def _schedule_beacon(self, b: SimBeacon, t_start: float) -> None:
        """First delivery to each gateway after a gossip delay, then an anti-entropy repeat now and then."""
        for gw in self.gws:
            first = t_start + self.rng.expovariate(1 / (1.5 + 0.8 * b.depth))
            self.at(first, lambda gw=gw, b=b: self._deliver(gw, b))

            def repeat(gw=gw, b=b):
                if b.id in self.faults.get('killed', {}) and self.t >= self.faults['killed'][b.id] + 60:
                    self._deliver(gw, b, silent=True)       # neighbours now flag it ORIGIN_SILENT
                else:
                    self._deliver(gw, b)
                self.at(self.t + self.rng.uniform(40, 90), repeat)
            self.at(first + self.rng.uniform(30, 90), repeat)

    # ------------------------------------------------------------------ robot
    def _tree_path(self, a: int, b: int) -> list:
        def up(x):
            out, seen = [x], {x}
            while self.beacons[out[-1]].link is not None and self.beacons[out[-1]].link in self.beacons:
                nx = self.beacons[out[-1]].link
                if nx in seen:
                    break
                out.append(nx)
                seen.add(nx)
            return out
        ua, ub = up(a), up(b)
        common = next((x for x in ua if x in set(ub)), ua[-1])
        return ua[:ua.index(common) + 1] + list(reversed(ub[:ub.index(common)]))

    def _plan_next(self) -> None:
        if self.targets:
            tgt = self.targets.pop(0)
            if tgt not in self.beacons:
                self.log(f'[zonesim] target #{tgt} unknown, skipped')
                return self._plan_next()
            path = self._tree_path(self.last_beacon, tgt)
            self.route = [self.beacons[i].drop.copy() for i in path[1:]]
            self.route_ids = path[1:]
            self.current_target = tgt
            self.phase = 'GOTO'
        else:
            path = self._tree_path(self.last_beacon, min(self.beacons))
            self.route = [self.beacons[i].drop.copy() for i in path[1:]]
            self.route_ids = path[1:]
            self.current_target = None
            self.phase = 'RETURN'

    def _robot_step(self, dt: float) -> None:
        if self.phase == 'WAIT':
            if not self.wait_briefing or self.t > self.briefing_deadline:
                if not self.targets and not self.mission_id:
                    # every danger, victims first (resources and searched-area markers are not targets)
                    self.targets = [b.id for b in sorted(self.beacons.values(),
                                                         key=lambda b: (b.kind != 'VICTIM', b.depth))
                                    if b.kind in TARGET_KINDS]
                    self.total = len(self.targets)
                    self.plan_set = set(self.targets)
                    self.log(f'[zonesim] no briefing heard: the Executor takes every hazard ({self.targets})')
                self._plan_next()
            return
        if self.phase == 'TREAT':
            if self.t >= self.treat_until:
                if self.plan_set is None or self.current_target in self.plan_set:
                    self.done += 1
                self.treated.append(self.current_target)
                self._plan_next()
            return
        if self.phase == 'DONE':
            return
        if not self.route:
            if self.phase == 'RETURN':
                self.phase = 'DONE'
                self.finished = True
                self.log(f'[zonesim] robot back at the exit: {self.done}/{self.total} treated')
            else:
                self.phase = 'TREAT'
                self.treat_until = self.t + 5.0
            return
        tgt = self.route[0]
        d = tgt - self.pos
        dist = float(np.linalg.norm(d))
        step = self.v * dt
        if dist <= step:
            self.pos = tgt.copy()
            self.route.pop(0)
            self.last_beacon = self.route_ids.pop(0)
        else:
            self.pos = self.pos + d / dist * step
            dist = step
        self.yaw = math.atan2(d[1], d[0])
        self.odo += dist
        self.slam_err += self.nrng.normal(0, 0.01 * math.sqrt(max(dist, 1e-9)), 2)

    def _ping(self) -> None:
        self.seq = (self.seq + 1) & 0xFFFF
        self.truth[self.seq] = self.pos.copy()
        rep = self.pos + self.slam_err
        for tt, (dx, dy) in self.faults.get('slip', []):
            if self.t >= tt:
                rep = rep + np.array([dx, dy])
        fr = lmb2.encode_robot({'robot_id': self.robot_id, 'seq': self.seq,
                                'role': 'EXECUTOR' if self.robot_kind == 'executor' else 'WRITER',
                                'phase': self.phase if self.phase in lmb2.PHASES['EXECUTOR'] else 'GOTO',
                                'x': rep[0], 'y': rep[1], 'yaw': self.yaw, 'pose_sd_m': 0.15 + 0.01 * self.odo ** 0.5,
                                'odo_m': self.odo, 'battery_pct': max(0, 100 - int(self.t / 30)), 'done': self.done,
                                'total': self.total, 'ts': int(self.now_unix()), 'mission_id': self.mission_id,
                                'mission_ack': self.mission_ack, 'last_beacon': self.last_beacon},
                               self.key, self.net)
        ant = np.array([self.pos[0], self.pos[1], 0.30])
        for gw, g in self.gws.items():
            if self.gw_down(gw):
                continue
            true_r = float(np.linalg.norm(ant - g))
            idx = walls_on_path(self.pos, g[:2], self.boxes)
            walls = len(idx)
            # every wall has its own extra loss (4-9 dB) and extra time-of-flight range (0.2-1.0 m), the same
            # at every ping (as on a real site). The ONA only knows the averages: its NLOS correction is
            # deliberately imperfect here.
            wall_db = sum(self.wall_props[i][0] for i in idx)
            rssi = 14 + 2 - (31 + 30 * math.log10(max(true_r, 1))) - wall_db + self.rng.gauss(0, 4.0)
            if rssi < -123:
                continue
            r = true_r + sum(self.wall_props[i][1] for i in idx) + self.rng.gauss(0, self.range_sigma)
            self.emit({'gw': gw, 'ev': 'rx', 'frame': fr.hex(), 'rssi': round(rssi, 1), 'snr': 5.0,
                       'range_m': round(max(0.1, r), 2), 'range_sd': 0.5, 'range_src': 'tof', 'walls': walls})
        self.at(self.t + self.ping_s, self._ping)

    def _heartbeats(self) -> None:
        for gw, st in self.stats.items():
            if not self.gw_down(gw):
                hb = {'gw': gw, 'ev': 'hb', 'known': len(st['known']), 'rx_ok': st['rx'], 'duty': 0.004}
                if self.advertise_speed:
                    hb['sim_speed'] = self.speed       # live: the ONA's clock is real time, the mission runs faster
                self.emit(hb)
        self.at(self.t + 4.0, self._heartbeats)

    def _forge(self, gw: str) -> None:
        g = self.cal.local_to_gps(np.array([1.5, -0.5, 0.0]))
        rec = {'beacon_id': 900, 'seq': 1, 'kind': 'VICTIM', 'gps': {'lat': g['lat'], 'lon': g['lon'], 'err_m': 0.5},
               'severity': 90, 'confidence': 1.0, 'ts': int(self.now_unix())}
        fr = lmb2.encode_record(rec, self.key, self.net)
        self.log(f'[zonesim] {gw} injects a forged VICTIM #900')
        for k in range(4):
            self.at(self.t + 20 * k, lambda: self.emit({'gw': gw, 'ev': 'rx', 'frame': fr.hex(), 'rssi': -70.0}))

    # ------------------------------------------------------------------ downlink
    def receive_downlink(self, frame_hex: str) -> None:
        """A briefing frame transmitted by the ONA's gateways."""
        try:
            m = lmb2.decode_mission(bytes.fromhex(frame_hex), self.key, self.net)
        except (ValueError, lmb2.DecodeError) as e:
            self.log(f'[zonesim] Executor ignores a downlink frame: {e}')
            return
        if self.robot_kind != 'executor' or (m['robot_id'] is not None and m['robot_id'] != self.robot_id):
            return
        whole = self.asm.add(m)
        if whole is None or whole['mission_id'] == self.mission_id:
            return
        self.mission_id = whole['mission_id']
        self.mission_ack = True
        if whole['abort']:
            self.targets = []
        else:
            self.targets = [t for t in whole['targets'] if t in self.beacons and t not in self.treated]
        # a briefing replaces the plan: progress restarts on the new target list
        self.plan_set = set(self.targets)
        self.done = 0
        self.total = len(self.targets)
        self.log(f'[zonesim] Executor briefed: mission {self.mission_id}, targets {self.targets}')
        if self.phase in ('WAIT', 'DONE', 'RETURN', 'GOTO'):
            self.finished = False
            self._plan_next()

    # ------------------------------------------------------------------ run
    def start(self) -> None:
        for b in sorted(self.beacons.values(), key=lambda b: b.id):
            self._schedule_beacon(b, 0.2 + 0.05 * b.id)
        self.at(0.5, self._ping)
        self.at(0.1, self._heartbeats)
        for gw, tf in self.faults.get('forge', {}).items():
            self.at(tf, lambda gw=gw: self._forge(gw))

    def step_until(self, t_end: float, dt: float = 0.1, realtime: bool = False, poll=None) -> None:
        while self.t < t_end:
            t_next = self.t + dt
            while self.events and self.events[0][0] <= t_next:
                te, _, fn = heapq.heappop(self.events)
                self.t = max(self.t, te)
                fn()
            self._robot_step(t_next - self.t if t_next > self.t else dt)
            self.t = t_next
            if poll:
                poll()
            if realtime:
                time.sleep(dt / self.speed)


def parse_faults(a) -> dict:
    f = {'down': {}, 'forge': {}, 'corrupt': {}, 'killed': {}, 'slip': []}
    if a.liar:
        f['liar'] = a.liar
    for s in a.forge:
        gw, t = s.split('@')
        f['forge'][gw] = float(t)
    for s in a.corrupt:
        gw, t = s.split('@')
        f['corrupt'][gw] = float(t)
    for s in a.gw_down:
        gw, w = s.split('@')
        x, y = w.split('-')
        f['down'].setdefault(gw, []).append((float(x), float(y)))
    for s in a.kill:
        i, t = s.split('@')
        f['killed'][int(i)] = float(t)
    for s in a.slam_slip:
        t, d = s.split(':')
        dx, dy = d.split(',')
        f['slip'].append((float(t), (float(dx), float(dy))))
    return f


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='python -m ona.zonesim', description=__doc__.split('\n')[1])
    ap.add_argument('--config', default=os.path.join(HERE, '..', 'ona_config.json'))
    ap.add_argument('--mission', help='living-map-mission/1 file (default: built-in building)')
    ap.add_argument('--world', help='Gazebo .world file for the walls (with --mission)')
    ap.add_argument('--ona', default='127.0.0.1:47100', help='ONA UDP address')
    ap.add_argument('--stdout', action='store_true', help='print the gateway lines instead of sending them')
    ap.add_argument('--speed', type=float, default=2.0, help='simulated seconds per real second')
    ap.add_argument('--minutes', type=float, default=8.0)
    ap.add_argument('--robot', choices=['executor', 'writer'], default='executor')
    ap.add_argument('--no-wait', action='store_true', help="don't wait for a briefing: go for every hazard")
    ap.add_argument('--briefing-wait', type=float, default=120.0,
                    help='mission seconds the Executor waits for a briefing before taking every hazard (default 120)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--liar')
    ap.add_argument('--forge', action='append', default=[])
    ap.add_argument('--corrupt', action='append', default=[])
    ap.add_argument('--gw-down', action='append', default=[])
    ap.add_argument('--kill', action='append', default=[])
    ap.add_argument('--slam-slip', action='append', default=[])
    a = ap.parse_args(argv)

    with open(a.config, encoding='utf-8') as f:
        cfg_d = json.load(f)
    if a.mission:
        with open(a.mission, encoding='utf-8') as f:
            ms = json.load(f)
        if ms.get('anchor'):
            cfg_d['anchor'] = {**cfg_d.get('anchor', {}), **ms['anchor']}
        sc = {'records': ms['beacons'], 'home': ms.get('home', [0, 0]), 'boxes': []}
        wf = a.world or ms.get('world')
        if wf and not os.path.isabs(wf) and not os.path.exists(wf):
            wf = os.path.join(os.path.dirname(os.path.abspath(a.mission)), wf)
        if wf and os.path.exists(wf):
            sc['boxes'] = load_world_boxes(wf)
    else:
        sc = builtin_scenario()
    cfg = OnaConfig.from_dict(cfg_d)
    host, port = a.ona.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    addr = (host, int(port))

    def sink(d):
        line = json.dumps(d, separators=(',', ':'))
        if a.stdout:
            print(line, flush=True)
        else:
            try:
                sock.sendto(line.encode(), addr)
            except OSError:
                pass

    sim = ZoneSim(cfg, sc, seed=a.seed, speed=a.speed, sink=sink, robot=a.robot, wait_briefing=not a.no_wait,
                  faults=parse_faults(a), log=lambda s: print(s, file=sys.stderr, flush=True))
    sim.advertise_speed = not a.stdout
    sim.briefing_deadline = a.briefing_wait

    def poll():
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                return
            for ln in data.decode('utf-8', 'replace').splitlines():
                try:
                    d = json.loads(ln)
                except json.JSONDecodeError:
                    continue
                if d.get('ev') == 'tx' and d.get('frame'):
                    sim.receive_downlink(d['frame'])

    print(f'[zonesim] {sc.get("name", a.mission)}: {len(sim.beacons)} beacons, {len(sim.boxes)} walls, '
          f'gateways {", ".join(sim.gws)}, robot {a.robot}, {a.speed}x -> {"stdout" if a.stdout else a.ona}',
          file=sys.stderr, flush=True)
    if not a.no_wait:
        print('[zonesim] the Executor waits at the exit for a briefing (dispatch a mission on the dashboard; '
              f'after {sim.briefing_deadline:.0f} s it takes every hazard)', file=sys.stderr, flush=True)
    sim.start()
    try:
        sim.step_until(a.minutes * 60, realtime=True, poll=poll)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
