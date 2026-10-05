"""
ona_radio.py - what the three gateways of the Outside Network Area (ONA) hear
in the Gazebo world (no ROS inside, tested in tools/test_ona_link.py).

The ONA stands outside the contaminated building: three LoRa gateways around
it, a small computer that checks and votes on what they hear, and an uplink
(LTE, satellite as backup) to the Command Post. In the simulation the
gateways are virtual: this module computes, from the true position of the
robot in Gazebo and the walls of the world, what each gateway would receive:

  * a ROBOT frame ("ping", 44 bytes, signed) every couple of seconds with the
    robot's OWN idea of where it is (its SLAM pose), and, for each gateway
    that hears it, the RSSI and the two-way time-of-flight DISTANCE. Every wall
    on the way costs 4 to 9 dB and makes the distance 0.2 to 1.0 m too long
    (each wall its own, always the same: like a real building). The ONA turns
    the three distances into a position: three spheres around three known
    antennas, like GPS satellites.
  * the beacons' RECORD frames, as the beacon mesh carries them to the
    gateways (straight from the beacon when it is close enough, else relayed
    by the beacon nearest to the gateway), and repeated now and then.
  * the ONA's answer: MISSION frames (briefings for the Executor), which the
    robot hears from any gateway its radio reaches.

The ONA receives one JSON line per frame and gateway, exactly as the real
gateway firmware prints them (living_map/beacon_net/firmware/gateway_node).
"""
import json
import math
import random
import xml.etree.ElementTree as ET

from writer_robot import ona_lmb2 as lmb2

# Gazebo models that are not walls (beacons, robots, the hazard blocks)
NOT_WALLS = ('beacon', 'ground', 'writer', 'executor', 'treated', 'radiation', 'fire', 'gas', 'victim',
             'sun', 'light', 'deco')
TX_DBM, ANT_DBI = 14.0, 2.0
SENSITIVITY_DBM = -123.0          # SF7..SF9, 125 kHz: what a gateway can still decode
DIRECT_MIN_DBM = -118.0           # a beacon this weak is reached through the mesh instead


# ---------------------------------------------------------------------- walls
def load_world_boxes(path):
    """Static boxes of an SDF world (walls, rubble) as (cx, cy, sx, sy, yaw)."""
    boxes = []
    root = ET.parse(path).getroot()
    for m in root.iter('model'):
        name = m.get('name', '').lower()
        if name.startswith(NOT_WALLS):
            continue
        pose = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        pose += [0.0] * (6 - len(pose))
        size = None
        for b in m.iter('box'):
            size = [float(v) for v in (b.findtext('size') or '').split()]
            break
        if not size or len(size) < 2:
            continue
        for link in m.iter('link'):
            lp = link.findtext('pose')
            if lp:
                v = [float(x) for x in lp.split()] + [0.0] * 6
                pose[0] += v[0]
                pose[1] += v[1]
                pose[5] += v[5]
            break
        boxes.append((pose[0], pose[1], size[0], size[1], pose[5]))
    return boxes


def seg_hits_box(p, q, box):
    """Does the segment p-q cross the oriented rectangle box = (cx, cy, sx, sy, yaw)?"""
    cx, cy, sx, sy, yaw = box
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


def seg_box_chord(p, q, box):
    """Length (m) of the part of segment p-q inside the oriented rectangle (0 if it misses it)."""
    cx, cy, sx, sy, yaw = box
    c, s = math.cos(-yaw), math.sin(-yaw)
    x0, y0 = (p[0] - cx) * c - (p[1] - cy) * s, (p[0] - cx) * s + (p[1] - cy) * c
    x1, y1 = (q[0] - cx) * c - (q[1] - cy) * s, (q[0] - cx) * s + (q[1] - cy) * c
    hx, hy = sx / 2, sy / 2
    t0, t1 = 0.0, 1.0
    dx, dy = x1 - x0, y1 - y0
    for pp, qq in ((-dx, x0 + hx), (dx, hx - x0), (-dy, y0 + hy), (dy, hy - y0)):
        if abs(pp) < 1e-12:
            if qq < 0:
                return 0.0
        else:
            r = qq / pp
            if pp < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return 0.0
    return (t1 - t0) * math.hypot(q[0] - p[0], q[1] - p[1])


def load_config(path):
    """ONA config (the same file the ONA reads): key, network id, gateways in the map frame."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    gws = {}
    for g, v in d.get('gateways', {}).items():
        if 'local' not in v:
            raise ValueError(f'gateway {g}: the robots need its "local" (map frame) position')
        x, y, z = (list(v['local']) + [0.0, 0.0, 2.0])[:3]
        gws[g] = (float(x), float(y), float(z))
    return {'key': lmb2.parse_key(d.get('key', 'demo')), 'net_id': int(d.get('net_id', lmb2.NET_DEFAULT)),
            'gateways': gws, 'anchor': d.get('anchor', {}), 'name': d.get('name', 'ONA'),
            'radio': d.get('radio', {})}


# ---------------------------------------------------------------------- radio
class GatewayRadio:
    """The three (or more) virtual gateways around the building."""

    def __init__(self, gateways, boxes, seed=1, range_sigma=0.35, wall_db=(4.0, 9.0), wall_bias=(0.2, 1.0),
                 robot_height=0.30, beacon_height=0.05, rock_db_per_m=None, tof_needs_los=False):
        # UNDERGROUND (rock_db_per_m set, the Gafsa mine): the boxes are rock, metres thick. A radio
        # wave crossing rock loses rock_db_per_m for every metre of it (a 4 m pillar ~ 30 dB) and no
        # time-of-flight range can be measured through it (tof_needs_los): a gateway measures a
        # distance only when it sees the robot straight down a gallery.
        self.gws = dict(gateways)            # name -> (x, y, z) antenna, map frame
        self.boxes = list(boxes)
        self.rng = random.Random(seed)
        wr = random.Random(seed + 1000)
        # every wall its own loss and its own extra time of flight, the same at every ping
        self.wall_props = [(wr.uniform(*wall_db), wr.uniform(*wall_bias)) for _ in self.boxes]
        self.range_sigma = range_sigma
        self.robot_h = robot_height
        self.beacon_h = beacon_height
        self.rock_db = rock_db_per_m
        self.tof_los = bool(tof_needs_los)

    def walls(self, a, b):
        return [i for i, bx in enumerate(self.boxes) if seg_hits_box(a, b, bx)]

    def _loss(self, idx, a, b):
        if self.rock_db is None:
            return sum(self.wall_props[i][0] for i in idx)
        return sum(self.rock_db * seg_box_chord(a, b, self.boxes[i]) for i in idx)

    def rssi(self, xy, gw, h=None, noise=True):
        """Received power (dBm) at gateway gw from a transmitter at xy (map frame)."""
        g = self.gws[gw]
        h = self.robot_h if h is None else h
        d = math.sqrt((xy[0] - g[0]) ** 2 + (xy[1] - g[1]) ** 2 + (h - g[2]) ** 2)
        loss = self._loss(self.walls(xy, g[:2]), xy, g[:2])
        p = TX_DBM + ANT_DBI - (31.0 + 30.0 * math.log10(max(d, 1.0))) - loss
        return p + (self.rng.gauss(0.0, 4.0) if noise else 0.0)

    def hears(self, xy, gw, h=None):
        return self.rssi(xy, gw, h, noise=False) >= SENSITIVITY_DBM + 3.0

    def ping_lines(self, frame, true_xy):
        """One ROBOT frame -> the line of every gateway that decodes it (with its ranging)."""
        out = []
        hexf = frame.hex()
        for gw, g in self.gws.items():
            true_r = math.sqrt((true_xy[0] - g[0]) ** 2 + (true_xy[1] - g[1]) ** 2 + (self.robot_h - g[2]) ** 2)
            idx = self.walls(true_xy, g[:2])
            rssi = (TX_DBM + ANT_DBI - (31.0 + 30.0 * math.log10(max(true_r, 1.0)))
                    - self._loss(idx, true_xy, g[:2]) + self.rng.gauss(0.0, 4.0))
            if rssi < SENSITIVITY_DBM:
                continue
            line = {'gw': gw, 'ev': 'rx', 'frame': hexf, 'rssi': round(rssi, 1),
                    'snr': round(max(-15.0, min(10.0, (rssi + 117.0) / 2.0)), 1)}
            if not (self.tof_los and idx):
                bias = 0.0 if self.rock_db is not None else sum(self.wall_props[i][1] for i in idx)
                r = true_r + bias + self.rng.gauss(0.0, self.range_sigma)
                line.update({'range_m': round(max(0.1, r), 2), 'range_sd': 0.5, 'range_src': 'tof'})
            out.append(line)
        return out

    def record_line(self, gw, frame, origin_xy, origin_id, relays):
        """A beacon's record as gateway gw receives it. relays: {beacon id: (x, y)} already on the floor."""
        fr = bytearray(frame)
        rssi = self.rssi(origin_xy, gw, self.beacon_h)
        relay, hops = origin_id, 0
        if rssi < DIRECT_MIN_DBM and relays:
            # too far / too many walls: the mesh carries it to the beacon that the gateway hears best
            best = max(((self.rssi(p, gw, self.beacon_h, noise=False), b) for b, p in relays.items()
                        if b != origin_id), default=None)
            if best is not None and best[0] > rssi:
                relay, hops = best[1], 1 + int(math.hypot(origin_xy[0] - relays[best[1]][0],
                                                          origin_xy[1] - relays[best[1]][1]) // 6.0)
                rssi = best[0] + self.rng.gauss(0.0, 3.0)
        if rssi < SENSITIVITY_DBM:
            rssi = SENSITIVITY_DBM + self.rng.uniform(0.5, 3.0)   # the mesh gets it there in the end
        fr[2:4] = int(relay & 0xFFFF).to_bytes(2, 'little')         # relay id, hops: not signed
        fr[4] = (min(15, hops) << 4) | 15
        fr[5] = 0
        return {'gw': gw, 'ev': 'rx', 'frame': bytes(fr).hex(), 'rssi': round(rssi, 1),
                'snr': round(max(-15.0, min(10.0, (rssi + 117.0) / 2.0)), 1)}


# ---------------------------------------------------------------------- records
def retime_frames(frames, key, net_id, newest_age_s=120.0, now=None):
    """Re-sign a saved mission so that its newest record is `newest_age_s` old: a mission file from
    yesterday would show every gas leak as faded away on the dashboard."""
    import time as _t
    now = _t.time() if now is None else now
    recs = []
    for fr in frames:
        try:
            recs.append(lmb2.decode_record(bytes(fr), key, net_id))
        except lmb2.DecodeError:
            recs.append(None)
    valid = [r for r in recs if r is not None]
    if not valid:
        return list(frames)
    shift = (now - newest_age_s) - max(r['ts'] for r in valid)
    out = []
    for fr, r in zip(frames, recs):
        if r is None:
            out.append(bytes(fr))
            continue
        r = dict(r, ts=int(r['ts'] + shift))
        r.pop('_hdr', None)
        out.append(lmb2.encode_record(r, key, net_id))
    return out


def briefing_from_frames(asm, frame, key, net_id, robot_id):
    """A MISSION frame heard on the air -> the whole briefing when complete (else None).
    Raises lmb2.DecodeError for a frame that is not a valid signed briefing."""
    m = lmb2.decode_mission(bytes(frame), key, net_id)
    if m['robot_id'] is not None and m['robot_id'] != robot_id:
        return None
    whole = asm.add(m)
    if whole is None:
        return None
    return {'mission_id': whole['mission_id'], 'seq': whole['seq'], 'targets': list(whole['targets']),
            'return_to_exit': whole['return_to_exit'], 'abort': whole['abort'], 'in_order': whole['in_order'],
            'ts': whole['ts']}
