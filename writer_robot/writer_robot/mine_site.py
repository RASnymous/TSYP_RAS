"""
mine_site.py - the GAFSA MINE scenario: what the robots' mine sensor suite
measures (no ROS inside: used by mine_sensors_node and by the 2D simulators).

In the building scenario the camera recognises coloured blocks. Underground the
dangers are mostly INVISIBLE (gas, radon, an unstable roof) or DARK (a fire, a
person), and the robot must also MARK RESOURCES. A mine world
(worlds/gafsa_mine.world) therefore comes with a site file (gafsa_mine.json, made
by tools/make_mine.py) that lists them, and this module turns the robot's true
pose into what each real instrument would report:

  multi-gas detector  CO, NO2, H2S, CH4 (% of the lower explosive limit) and O2.
                      Gas spreads ALONG THE GALLERIES: concentration = peak x
                      exp(-d / spread), d = the walking distance from the source
                      through the galleries, never through rock.
                      Danger = the high alarm of a portable detector: CO 100 ppm,
                      NO2 5 ppm, H2S 10 ppm, CH4 20 %LEL, O2 below 19.5 %.
  radon monitor       Bq/m3, the same spreading model (radon builds up in dead
                      ends without ventilation). Danger = 1000 Bq/m3 (IAEA
                      workplace level); 300 Bq/m3 (EU reference level) is logged.
  gamma probe         uSv/h: background, higher near phosphate (the ore carries
                      40-60 ppm uranium). Not a danger here: it helps grade ore.
  thermal camera      57 degree view (FLIR Lepton class): a hot spot (fire) up to
                      8 m, a body at 30-37 C up to 6 m, in line of sight.
  roof scanner        an upward depth camera / tilted LiDAR: roof sag and cracks
                      up to 3.5 m ahead (120 degree view), in line of sight.
  wall probe          gamma spectrometer + camera aimed at the wall: a phosphate
                      seam within 2 m; grade in % P2O5 (+- 1 %).
  camera + XRF        mineral finds (gold, gemstones) up to 3 m in the camera's
                      view, in line of sight.
  victim search       the thermal camera also "paints" every patch of floor it
                      has looked at (57 degrees, 6 m, line of sight). The site
                      file cuts the mine into search areas; at 85 % coverage an
                      area is SEARCHED and gets a beacon: "searched, N victims"
                      (the digital version of the X marking rescue teams spray
                      at a door). Value = the coverage.

Each detection is reported like the vision node reports a block: an event type,
the estimated position IN THE ROBOT'S MAP FRAME (computed from the measured range
and bearing, applied to the robot's own pose), the range, and a value 0..1 for
the record's severity (for phosphate: the grade itself, see mission_log).
For gas and radon, the robot does not "see" a source: it reports the source it
localises from the concentration gradient; the simulation gives the true source
with a 0.25 m error.
"""
import heapq
import json
import math
import os
import random
import xml.etree.ElementTree as ET

HAZARD_TYPES = ('gas', 'radiation', 'fire', 'structural', 'victim')
RESOURCE_TYPES = ('phosphate', 'gold', 'gemstone')
NOT_AVOIDED = RESOURCE_TYPES + ('searched',)   # marked with a beacon, never a keep-out zone
EVENT_TYPE = {'radon': 'radiation', 'gas': 'gas', 'fire': 'fire', 'roof': 'structural', 'victim': 'victim',
              'phosphate': 'phosphate', 'gold': 'gold', 'gemstone': 'gemstone'}
THERMAL_HFOV = math.radians(57.0)
CAMERA_HFOV = 1.089
DEDUP_M = {'phosphate': 4.0}           # one record per 4 m of seam; else one per 1.5 m
# what a record's severity (0..255) means underground, for the Command Post (value = severity / 100):
# gas: x the alarm level = sev / 20; radon Bq/m3 = sev x 50; fire C = sev x 6; roof sag mm = sev x 0.6;
# searched: % of the area covered = sev; phosphate: % P2O5 = sev / 4 (mission_log)


def grade_class(p2o5):
    """The basin's usual grade classes."""
    return 'rich' if p2o5 > 27 else ('medium' if p2o5 >= 22 else ('poor' if p2o5 >= 15 else 'waste'))


def site_file_for(world_path):
    """worlds/x.world -> worlds/x.json if it exists (a mine site), else None."""
    if not world_path:
        return None
    p = os.path.splitext(world_path)[0] + '.json'
    return p if os.path.exists(p) else None


def load_collision_boxes(world_path):
    """Static collision boxes of an SDF world: (cx, cy, sx, sy, yaw, name)."""
    out = []
    root = ET.parse(world_path).getroot()
    for m in root.iter('model'):
        name = m.get('name', '')
        if m.findtext('static', 'false').strip() != 'true' or name == 'ground_plane':
            continue
        pose = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()] + [0.0] * 6
        for col in m.iter('collision'):
            box = col.find('geometry/box/size')
            if box is None:
                continue
            sx, sy, _ = [float(v) for v in box.text.split()]
            cp = [float(v) for v in (col.findtext('pose') or '0 0 0 0 0 0').split()] + [0.0] * 6
            yaw = pose[5] + cp[5]
            cx = pose[0] + cp[0] * math.cos(pose[5]) - cp[1] * math.sin(pose[5])
            cy = pose[1] + cp[0] * math.sin(pose[5]) + cp[1] * math.cos(pose[5])
            out.append((cx, cy, sx, sy, yaw, name))
    return out


def seg_hits_box(p, q, box, shrink=0.0):
    """Does the segment p-q cross the oriented rectangle (cx, cy, sx, sy, yaw, ...)?"""
    cx, cy, sx, sy, yaw = box[:5]
    c, s = math.cos(-yaw), math.sin(-yaw)
    x0, y0 = (p[0] - cx) * c - (p[1] - cy) * s, (p[0] - cx) * s + (p[1] - cy) * c
    x1, y1 = (q[0] - cx) * c - (q[1] - cy) * s, (q[0] - cx) * s + (q[1] - cy) * c
    hx, hy = sx / 2 - shrink, sy / 2 - shrink
    if hx <= 0 or hy <= 0:
        return False
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


class Detection:
    __slots__ = ('etype', 'x', 'y', 'range', 'bearing', 'value', 'item', 'detail', 'grade')

    def __init__(self, etype, x, y, rng, bearing, value, item, detail='', grade=None):
        self.etype, self.x, self.y, self.range, self.bearing = etype, x, y, rng, bearing
        self.value, self.item, self.detail, self.grade = value, item, detail, grade

    def as_tuple(self):
        return self.etype, self.x, self.y, self.range


class MineSite:
    def __init__(self, site_path, world_path=None, res=0.1, seed=1):
        with open(site_path, encoding='utf-8') as f:
            self.site = json.load(f)
        if world_path is None:
            world_path = os.path.join(os.path.dirname(site_path), self.site.get('world', ''))
        self.boxes = load_collision_boxes(world_path)
        self.items = self.site['items']
        self.th = self.site.get('thresholds', {})
        self.bg = self.site.get('background', {})
        self.rng = random.Random(seed)
        self.res = res
        self._grid()
        self.fields = {}
        for it in self.items:
            if it['type'] in ('gas', 'radon'):
                self.fields[it['id']] = self._walk_distance(it['x'], it['y'])
        self._zones()

    # ---------------------------------------------------------- geometry
    def _grid(self):
        xs, ys = [], []
        for (cx, cy, sx, sy, yaw, _) in self.boxes:
            c, s = math.cos(yaw), math.sin(yaw)
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
                xs.append(cx + c * dx - s * dy)
                ys.append(cy + s * dx + c * dy)
        self.x0, self.y0 = min(xs) - 0.5, min(ys) - 0.5
        self.W = int((max(xs) + 0.5 - self.x0) / self.res) + 1
        self.H = int((max(ys) + 0.5 - self.y0) / self.res) + 1
        blocked = bytearray(self.W * self.H)
        for (cx, cy, sx, sy, yaw, name) in self.boxes:
            c, s = math.cos(yaw), math.sin(yaw)
            r = math.hypot(sx, sy) / 2 + self.res
            i0, i1 = max(0, int((cy - r - self.y0) / self.res)), min(self.H - 1, int((cy + r - self.y0) / self.res))
            j0, j1 = max(0, int((cx - r - self.x0) / self.res)), min(self.W - 1, int((cx + r - self.x0) / self.res))
            for i in range(i0, i1 + 1):
                py = self.y0 + (i + 0.5) * self.res - cy
                for j in range(j0, j1 + 1):
                    px = self.x0 + (j + 0.5) * self.res - cx
                    lx, ly = px * c + py * s, -px * s + py * c
                    if abs(lx) <= sx / 2 + 0.02 and abs(ly) <= sy / 2 + 0.02:
                        blocked[i * self.W + j] = 1
        self.blocked = blocked

    def _zones(self):
        """Search areas: zone index (1..n) of every free cell, the free cells per zone, coverage."""
        self.zones = list(self.site.get('zones', []))
        srch = self.site.get('search', {})
        self.search_hfov = math.radians(float(srch.get('hfov_deg', 57.0)))
        self.search_range = float(srch.get('range_m', 6.0))
        self.search_done = float(srch.get('done_fraction', 0.85))
        self.zone_of = bytearray(self.W * self.H)
        self.zone_total = [0] * len(self.zones)
        self.zone_seen = [0] * len(self.zones)
        self.zone_victims = [set() for _ in self.zones]
        self.zone_reported = set()
        self.seen = bytearray(self.W * self.H)
        for zi, z in enumerate(self.zones):
            for x0, x1, y0, y1 in z['rects']:
                for i in range(max(0, int((y0 - self.y0) / self.res)), min(self.H, int((y1 - self.y0) / self.res) + 1)):
                    cy = self.y0 + (i + 0.5) * self.res
                    if not (y0 <= cy <= y1):
                        continue
                    for j in range(max(0, int((x0 - self.x0) / self.res)), min(self.W, int((x1 - self.x0) / self.res) + 1)):
                        cx = self.x0 + (j + 0.5) * self.res
                        k = i * self.W + j
                        if x0 <= cx <= x1 and not self.blocked[k] and not self.zone_of[k]:
                            self.zone_of[k] = zi + 1
                            self.zone_total[zi] += 1

    def zone_index(self, x, y):
        """The search area that contains (x, y), or -1."""
        for zi, z in enumerate(self.zones):
            if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in z['rects']):
                return zi
        return -1

    def coverage(self):
        """[(zone id, name, fraction seen, victims found)] - the victim search so far."""
        return [(z['id'], z['name'], self.zone_seen[i] / max(1, self.zone_total[i]), len(self.zone_victims[i]))
                for i, z in enumerate(self.zones)]

    def _paint(self, x, y, yaw):
        """The thermal camera looks: mark the free cells it sees (rays every 2 degrees, stop at rock)."""
        if not self.zones:
            return
        n = max(2, int(math.degrees(self.search_hfov) / 2.0) + 1)
        W, res = self.W, self.res
        for r in range(n):
            a = yaw - self.search_hfov / 2 + self.search_hfov * r / (n - 1)
            ca, sa = math.cos(a), math.sin(a)
            d = 0.0
            while d <= self.search_range:
                c = self._cell(x + d * ca, y + d * sa)
                if c is None:
                    break
                k = c[0] * W + c[1]
                if self.blocked[k]:
                    break
                if not self.seen[k]:
                    self.seen[k] = 1
                    z = self.zone_of[k]
                    if z:
                        self.zone_seen[z - 1] += 1
                d += res * 0.7

    def _cell(self, x, y):
        j, i = int((x - self.x0) / self.res), int((y - self.y0) / self.res)
        if 0 <= i < self.H and 0 <= j < self.W:
            return i, j
        return None

    def _walk_distance(self, x, y):
        """Walking distance (m) from (x, y) to every free cell, through the galleries (Dijkstra)."""
        W, H, res = self.W, self.H, self.res
        dist = [math.inf] * (W * H)
        c = self._cell(x, y)
        if c is None:
            return dist
        k0 = c[0] * W + c[1]
        dist[k0] = 0.0
        heap = [(0.0, k0)]
        steps = [(-1, 0, res), (1, 0, res), (0, -1, res), (0, 1, res),
                 (-1, -1, res * 1.4142), (-1, 1, res * 1.4142), (1, -1, res * 1.4142), (1, 1, res * 1.4142)]
        blocked = self.blocked
        while heap:
            d, k = heapq.heappop(heap)
            if d > dist[k]:
                continue
            i, j = divmod(k, W)
            for di, dj, w in steps:
                ii, jj = i + di, j + dj
                if 0 <= ii < H and 0 <= jj < W:
                    kk = ii * W + jj
                    if blocked[kk]:
                        continue
                    nd = d + w
                    if nd < dist[kk]:
                        dist[kk] = nd
                        heapq.heappush(heap, (nd, kk))
        return dist

    def walk_dist(self, item_id, x, y):
        c = self._cell(x, y)
        f = self.fields.get(item_id)
        if c is None or f is None:
            return math.inf
        return f[c[0] * self.W + c[1]]

    def los(self, a, b, ignore=()):
        """Line of sight between two points: no rock (or other solid box) in between."""
        for box in self.boxes:
            if any(tag and tag in box[5] for tag in ignore):
                continue
            if seg_hits_box(a, b, box, shrink=0.03):
                return False
        return True

    # ---------------------------------------------------------- instruments
    def readings(self, x, y):
        """What the gas detector, the radon monitor and the gamma probe read at (x, y)."""
        r = {'co_ppm': 0.0, 'no2_ppm': 0.0, 'h2s_ppm': 0.0, 'ch4_lel': 0.0, 'o2_pct': 20.9,
             'radon_bqm3': float(self.bg.get('radon_bqm3', 60.0)), 'gamma_usvh': float(self.bg.get('gamma_usvh', 0.08))}
        for it in self.items:
            t = it['type']
            if t == 'gas':
                d = self.walk_dist(it['id'], x, y)
                f = math.exp(-d / float(it.get('spread_m', 2.0))) if math.isfinite(d) else 0.0
                g = it.get('gases', {})
                r['co_ppm'] += g.get('co_ppm', 0.0) * f
                r['no2_ppm'] += g.get('no2_ppm', 0.0) * f
                r['h2s_ppm'] += g.get('h2s_ppm', 0.0) * f
                r['ch4_lel'] += g.get('ch4_lel', 0.0) * f
                r['o2_pct'] -= g.get('o2_deficit_pct', 0.0) * f
            elif t == 'radon':
                d = self.walk_dist(it['id'], x, y)
                f = math.exp(-d / float(it.get('spread_m', 2.5))) if math.isfinite(d) else 0.0
                r['radon_bqm3'] += float(it.get('peak_bqm3', 1000.0)) * f
            elif t == 'phosphate':
                dd = _seg_dist(x, y, it['x0'], it['y0'], it['x1'], it['y1'])
                ore = float(self.bg.get('gamma_ore_usvh', 0.22)) * float(it.get('grade_p2o5', 25.0)) / 29.0
                r['gamma_usvh'] += ore * math.exp(-dd / 1.5)
        return r

    def danger(self, rd):
        """Which gas readings are above their danger level (list of text)."""
        th = self.th
        out = []
        for k, unit, lim in (('co_ppm', 'ppm CO', th.get('co_ppm', 100.0)), ('no2_ppm', 'ppm NO2', th.get('no2_ppm', 5.0)),
                             ('h2s_ppm', 'ppm H2S', th.get('h2s_ppm', 10.0)), ('ch4_lel', '%LEL CH4', th.get('ch4_lel', 20.0))):
            if rd[k] >= lim:
                out.append(f'{rd[k]:.0f} {unit}')
        if rd['o2_pct'] < th.get('o2_min_pct', 19.5):
            out.append(f'O2 {rd["o2_pct"]:.1f} %')
        return out

    def detect(self, x, y, yaw, est_pose=None):
        """Everything the sensor suite reports from the TRUE pose (x, y, yaw). Positions are returned
        in the frame of est_pose (the robot's own SLAM pose), like a real sensor: it measures a
        range and a bearing, the robot places them on its own map."""
        ex_, ey_, eyaw = est_pose if est_pose is not None else (x, y, yaw)
        rng = self.rng
        out = []

        def place(tx, ty, err_frac=0.03, err_bearing=0.02, err_abs=0.0):
            d = math.hypot(tx - x, ty - y)
            b = math.atan2(ty - y, tx - x) - yaw
            d2 = max(0.05, d * (1.0 + rng.gauss(0.0, err_frac)) + rng.gauss(0.0, err_abs))
            b2 = b + rng.gauss(0.0, err_bearing)
            return (ex_ + d2 * math.cos(eyaw + b2), ey_ + d2 * math.sin(eyaw + b2), d, math.atan2(math.sin(b), math.cos(b)))

        rd = self.readings(x, y)
        th = self.th
        self._paint(x, y, yaw)
        for it in self.items:
            t = it['type']
            et = EVENT_TYPE[t]
            if t == 'gas':
                bad = self.danger(rd)
                d = self.walk_dist(it['id'], x, y)
                if bad and math.isfinite(d) and d < 4.0 * float(it.get('spread_m', 2.0)):
                    px, py, dd, b = place(it['x'], it['y'], 0.0, 0.0, 0.25)
                    g = it.get('gases', {})
                    peak = max(g.get('co_ppm', 0.0) / th.get('co_ppm', 100.0), g.get('no2_ppm', 0.0) / th.get('no2_ppm', 5.0),
                               g.get('h2s_ppm', 0.0) / th.get('h2s_ppm', 10.0), g.get('ch4_lel', 0.0) / th.get('ch4_lel', 20.0))
                    out.append(Detection(et, px, py, dd, b, min(1.0, peak / 5.0), it['id'],
                                         'measured ' + ', '.join(bad)))
            elif t == 'radon':
                lim = th.get('radon_bqm3', 1000.0)
                if rd['radon_bqm3'] >= lim:
                    px, py, dd, b = place(it['x'], it['y'], 0.0, 0.0, 0.25)
                    out.append(Detection(et, px, py, dd, b, min(1.0, float(it.get('peak_bqm3', lim)) / (5.0 * lim)),
                                         it['id'], f'radon {rd["radon_bqm3"]:.0f} Bq/m3'))
            elif t in ('fire', 'victim'):
                rmax = 8.0 if t == 'fire' else 6.0
                d = math.hypot(it['x'] - x, it['y'] - y)
                b = math.atan2(it['y'] - y, it['x'] - x) - yaw
                b = math.atan2(math.sin(b), math.cos(b))
                if d <= rmax and abs(b) <= THERMAL_HFOV / 2 and self.los((x, y), (it['x'], it['y']), ignore=(it['id'],)):
                    px, py, dd, bb = place(it['x'], it['y'], 0.04, 0.02)
                    if t == 'fire':
                        out.append(Detection(et, px, py, dd, bb, min(1.0, float(it.get('temp_c', 300.0)) / 600.0),
                                             it['id'], f'hot spot {it.get("temp_c", 300):.0f} C'))
                    else:
                        zi = self.zone_index(it['x'], it['y'])
                        if zi >= 0:
                            self.zone_victims[zi].add(it['id'])
                        out.append(Detection(et, px, py, dd, bb, 0.6, it['id'],
                                             f'body heat {it.get("body_c", 34.0):.0f} C'))
            elif t == 'roof':
                d = math.hypot(it['x'] - x, it['y'] - y)
                b = math.atan2(it['y'] - y, it['x'] - x) - yaw
                b = math.atan2(math.sin(b), math.cos(b))
                if d <= 3.5 and abs(b) <= math.radians(60) and self.los((x, y), (it['x'], it['y'])):
                    px, py, dd, bb = place(it['x'], it['y'], 0.03, 0.03, 0.1)
                    out.append(Detection(et, px, py, dd, bb, min(1.0, float(it.get('sag_mm', 30.0)) / 60.0), it['id'],
                                         f'roof sag {it.get("sag_mm", 30):.0f} mm, cracks'))
            elif t == 'phosphate':
                qx, qy = _seg_closest(x, y, it['x0'], it['y0'], it['x1'], it['y1'])
                d = math.hypot(qx - x, qy - y)
                if 0.05 < d <= 2.0:
                    # look at a point 5 cm in front of the face (the face itself belongs to the rock)
                    k = (d - 0.05) / d
                    if self.los((x, y), (x + (qx - x) * k, y + (qy - y) * k)):
                        px, py, dd, bb = place(qx, qy, 0.02, 0.02)
                        grade = round(float(it['grade_p2o5']) + rng.gauss(0.0, 0.6), 1)
                        out.append(Detection(et, px, py, dd, bb, min(1.0, grade / 40.0), it['id'],
                                             f'{grade:.1f} % P2O5 ({grade_class(grade)}), layer {it.get("layer", "?")}',
                                             grade=grade))
            elif t in ('gold', 'gemstone'):
                d = math.hypot(it['x'] - x, it['y'] - y)
                b = math.atan2(it['y'] - y, it['x'] - x) - yaw
                b = math.atan2(math.sin(b), math.cos(b))
                if d <= 3.0 and abs(b) <= CAMERA_HFOV / 2 and self.los((x, y), (it['x'], it['y'])):
                    px, py, dd, bb = place(it['x'], it['y'], 0.02, 0.02)
                    what = 'gold (Au) signal on XRF' if t == 'gold' else 'crystal on camera, XRF to confirm'
                    out.append(Detection(et, px, py, dd, bb, 0.5, it['id'],
                                         what + (' - demonstration target' if it.get('demo') else '')))
        # victim search: an area just reached the coverage threshold -> one SEARCHED record for it
        for zi, z in enumerate(self.zones):
            if zi in self.zone_reported or self.zone_total[zi] == 0:
                continue
            frac = self.zone_seen[zi] / self.zone_total[zi]
            if frac >= self.search_done:
                self.zone_reported.add(zi)
                cx, cy = _zone_centre(z['rects'])
                d = math.hypot(cx - x, cy - y)
                b = math.atan2(cy - y, cx - x) - yaw
                # the area's centre, on the robot's own map (no sensor error: it is a map area)
                px = ex_ + d * math.cos(eyaw + b)
                py = ey_ + d * math.sin(eyaw + b)
                nv = len(self.zone_victims[zi])
                out.append(Detection('searched', px, py, d, math.atan2(math.sin(b), math.cos(b)), min(1.0, frac),
                                     z['id'], f'{z["name"]}: searched {100 * frac:.0f} %, '
                                              f'{nv} victim{"s" if nv != 1 else ""} found'))
        return out

    def hazards(self):
        """(event type, x, y, item id) of every hazard, for the simulators' checks and pictures."""
        out = []
        for it in self.items:
            if it['type'] in ('gas', 'radon', 'fire', 'roof', 'victim'):
                out.append((EVENT_TYPE[it['type']], it['x'], it['y'], it['id']))
        return out

    def resources(self):
        out = []
        for it in self.items:
            if it['type'] == 'phosphate':
                out.append(('phosphate', (it['x0'] + it['x1']) / 2, (it['y0'] + it['y1']) / 2, it['id']))
            elif it['type'] in ('gold', 'gemstone'):
                out.append((it['type'], it['x'], it['y'], it['id']))
        return out


def _zone_centre(rects):
    """Area-weighted centre of a zone's rectangles, snapped into the biggest one."""
    a = [(x1 - x0) * (y1 - y0) for x0, x1, y0, y1 in rects]
    big = rects[a.index(max(a))]
    return (big[0] + big[1]) / 2, (big[2] + big[3]) / 2


def _seg_closest(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return x0 + t * dx, y0 + t * dy


def _seg_dist(px, py, x0, y0, x1, y1):
    qx, qy = _seg_closest(px, py, x0, y0, x1, y1)
    return math.hypot(px - qx, py - qy)


class EventFilter:
    """The node's de-duplication: one beacon event per hazard / per 4 m of seam (like the vision node:
    one event per hazard within 1.5 m), and hazard sightings at most `rate_hz` per type."""

    def __init__(self, drop_range=4.5):
        self.marked = []           # (etype, x, y)
        self.drop_range = drop_range

    def new_event(self, det):
        r = DEDUP_M.get(det.etype, 1.5)
        if det.range > self.drop_range and det.etype not in ('gas', 'radiation', 'searched'):
            return False
        for et, mx, my in self.marked:
            if et == det.etype and math.hypot(det.x - mx, det.y - my) < r:
                return False
        self.marked.append((det.etype, det.x, det.y))
        return True
