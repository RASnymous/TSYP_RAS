"""
mission_log.py - the Writer's beacons as LMB2 records, saved as a mission file
(no ROS inside: used by mission_recorder, the Executor and the simulators).

For every beacon the Writer drops, the record it carries on the air is built
exactly like beacon_net's beacon_record_node does:

    kind      EXIT / WAYPOINT / RADIATION / THERMAL (fire) / GAS / ...
    position  the EVENT position for hazards (camera projection), the beacon
              itself for trail and exit beacons; map x/y -> lat/lon around
              an anchor
    err_m     0.3 m + 2 cm per metre driven (+25 % of the camera range)
    next      the way out: the beacon this one links to (id, distance and
              compass bearing measured from this beacon)
    ts, ttl, half-life   per kind (gas fades in minutes, a route for days)

and sealed into its 44-byte frame (HMAC with the mission key).

Mission file (JSON), written by the Writer, read by the Executor:
    {"format": "living-map-mission/1", "anchor": {...}, "net_id": 42,
     "key": "demo", "started": unix, "updated": unix, "world": "x.world",
     "home": [x, y],
     "beacons": [{"id", "event_type", "drop": [x, y], "event": [x, y],
                  "record": {...}, "frame": "<44 bytes hex>"}, ...]}

The Executor only uses the FRAMES (what it would receive over LoRa): it
verifies each one with the mission key and decodes it (decode_mission).
"""
import json
import math
import os
import time

from writer_robot import lmb2

M_PER_DEG_LAT = 111320.0
KIND_OF_EVENT = {'waypoint': 'WAYPOINT', 'trail': 'WAYPOINT', 'fire': 'THERMAL', 'thermal': 'THERMAL',
                 'radiation': 'RADIATION', 'victim': 'VICTIM', 'gas': 'GAS', 'obstruction': 'OBSTRUCTION',
                 'rubble': 'OBSTRUCTION', 'structural': 'STRUCTURAL', 'roof': 'STRUCTURAL', 'exit': 'EXIT',
                 'phosphate': 'PHOSPHATE', 'gold': 'GOLD', 'gemstone': 'GEMSTONE',
                 'searched': 'SEARCHED', 'clear': 'SEARCHED'}
HAZARD_KINDS = ('RADIATION', 'THERMAL', 'GAS', 'VICTIM', 'STRUCTURAL', 'OBSTRUCTION')
RESOURCE_KINDS = ('PHOSPHATE', 'GOLD', 'GEMSTONE')     # mine scenario: marked, never avoided
RESOURCE_CONF = {'PHOSPHATE': 0.85, 'GOLD': 0.6, 'GEMSTONE': 0.6}   # a find needs a sample to be sure


class GeoFrame:
    """map x/y (m) <-> lat/lon around an anchor; map +x points to compass
    `map_yaw_deg` measured from east (0 = east)."""

    def __init__(self, lat0=36.8065, lon0=10.1815, map_yaw_deg=0.0):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.yaw = math.radians(float(map_yaw_deg))

    def to_en(self, x, y):
        return (x * math.cos(self.yaw) - y * math.sin(self.yaw),
                x * math.sin(self.yaw) + y * math.cos(self.yaw))

    def from_en(self, e, n):
        return (e * math.cos(self.yaw) + n * math.sin(self.yaw),
                -e * math.sin(self.yaw) + n * math.cos(self.yaw))

    def to_latlon(self, x, y):
        e, n = self.to_en(x, y)
        return (self.lat0 + n / M_PER_DEG_LAT,
                self.lon0 + e / (M_PER_DEG_LAT * math.cos(math.radians(self.lat0))))

    def to_xy(self, lat, lon):
        n = (lat - self.lat0) * M_PER_DEG_LAT
        e = (lon - self.lon0) * M_PER_DEG_LAT * math.cos(math.radians(self.lat0))
        return self.from_en(e, n)

    def dict(self):
        return {'lat': self.lat0, 'lon': self.lon0, 'map_yaw_deg': round(math.degrees(self.yaw), 6)}


def key_bytes(key):
    return lmb2.DEMO_KEY if key in (None, '', 'demo') else lmb2.key_from_hex(key)


class MissionLog:
    def __init__(self, geo=None, key='demo', net_id=0x2A, err0=0.3, err_per_m=0.02, vision_conf=0.9,
                 clock=time.time):
        self.geo = geo or GeoFrame()
        self.key_name = key
        self.key = key_bytes(key)
        self.net = int(net_id)
        self.err0, self.err_k = float(err0), float(err_per_m)
        self.vconf = float(vision_conf)
        self.clock = clock
        self.entries = []
        self.drops = {}            # id -> (x, y)
        self.started = int(clock())
        self.home = None
        self.world = 'contaminated_zone.world'

    def add_drop(self, b, path_m=0.0):
        """b: a dropped beacon (beacon_id, event_type, x, y, link_id, optional
        event_x, event_y, range_m, value). path_m: distance driven so far.
        Returns the mission entry (record + frame), or None if invalid."""
        bid = int(b['beacon_id'])
        dx, dy = float(b['x']), float(b['y'])
        et = str(b.get('event_type', 'waypoint')).lower()
        kind = KIND_OF_EVENT.get(et, 'WAYPOINT')
        hazard = kind not in ('WAYPOINT', 'EXIT')
        ex, ey = dx, dy
        projected = b.get('event_x') is not None and b.get('event_y') is not None
        if projected:
            ex, ey = float(b['event_x']), float(b['event_y'])
        lat, lon = self.geo.to_latlon(ex, ey)
        err = self.err0 + self.err_k * float(path_m)
        if projected:
            err += 0.25 * float(b.get('range_m') or math.hypot(ex - dx, ey - dy))
        now = int(self.clock())
        if kind == 'PHOSPHATE' and b.get('grade') is not None:
            sev = int(round(max(0.0, min(63.75, float(b['grade']))) * 4))      # grade in 0.25 % P2O5 steps
        elif hazard:
            sev = int(round(100 * max(0.0, min(1.0, float(b.get('value') or 0.0)))))
        else:
            sev = 0
        rec = {
            'beacon_id': bid,
            'kind': kind,
            'gps': {'lat': round(lat, 7), 'lon': round(lon, 7), 'err_m': round(min(err, 25.4), 1), 'src': 'slam'},
            'severity': sev,
            'confidence': RESOURCE_CONF.get(kind, self.vconf) if hazard else 1.0,
            'ts': now,
        }
        link = b.get('link_id')
        if link is not None and int(link) in self.drops:
            px, py = self.drops[int(link)]
            e, n = self.geo.to_en(px - dx, py - dy)
            rec['next'] = {'id': int(link), 'dist_m': round(math.hypot(e, n), 1),
                           'bearing_deg': round(math.degrees(math.atan2(e, n)) % 360.0, 1)}
        try:
            r = lmb2.record_from_json(rec)
            frame = lmb2.encode(lmb2.Frame(rec=r, net_id=self.net, relay_id=bid), self.key)
        except lmb2.ProtoError:
            return None
        self.drops[bid] = (dx, dy)
        entry = {'id': bid, 'event_type': et, 'drop': [round(dx, 3), round(dy, 3)],
                 'event': [round(ex, 3), round(ey, 3)], 'record': rec, 'frame': frame.hex()}
        if b.get('detail'):
            entry['detail'] = str(b['detail'])[:120]     # what the instrument measured (not on the air)
        self.entries.append(entry)
        return entry

    def to_json(self):
        return {'format': 'living-map-mission/1', 'anchor': self.geo.dict(), 'net_id': self.net,
                'key': 'demo' if self.key == lmb2.DEMO_KEY else 'mission (not stored)',
                'started': self.started, 'updated': int(self.clock()),
                'world': self.world, 'home': self.home, 'beacons': self.entries}

    def save(self, path):
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.to_json(), f, indent=1)
        os.replace(tmp, path)


# ---------------------------------------------------------------- reading
def load_mission(path):
    with open(path) as f:
        m = json.load(f)
    if m.get('format') != 'living-map-mission/1':
        raise ValueError(f'{path}: not a living-map mission file')
    return m


def decode_mission(m, key='demo'):
    """Verify and decode every 44-byte frame of a mission (what the Executor
    receives over LoRa). Returns (records, rejected): records are
    lmb2.record_to_dict() dicts plus 'x', 'y' (map frame of the position)."""
    k = key_bytes(key)
    a = m.get('anchor', {})
    geo = GeoFrame(a.get('lat', 36.8065), a.get('lon', 10.1815), a.get('map_yaw_deg', 0.0))
    net = int(m.get('net_id', 0x2A))
    out, rejected = [], 0
    for e in m.get('beacons', []):
        try:
            f = lmb2.decode(bytes.fromhex(e['frame']), k, net)
        except (lmb2.ProtoError, ValueError, KeyError):
            rejected += 1
            continue
        d = lmb2.record_to_dict(f.rec)
        d['x'], d['y'] = geo.to_xy(d['gps']['lat'], d['gps']['lon'])
        out.append(d)
    return out, rejected, geo
