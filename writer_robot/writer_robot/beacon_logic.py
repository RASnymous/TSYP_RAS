"""
beacon_logic.py - where the Writer drops beacons, and how they chain together
(no ROS inside: used by beacon_drop_node and by the test simulators).

Each beacon points to one OLDER beacon: its `next`, "the way out". Following
`next` from any beacon leads, beacon after beacon, back to the EXIT beacon
dropped where the Writer entered. The beacons therefore form a tree rooted at
the exit. The Executor robot reverses these chains to reach an event, and
follows them to get out again.

For that to work, every `next` link must be a short, drivable, straight leg.
So a new beacon links to the last beacon the Writer DROPPED or DROVE PAST
(within `pass_radius`), never to one it left far behind:

  * EXIT beacon    dropped at the start pose (the root of the tree).
  * trail beacon   after a turn of more than `turn_deg` (and at least
                   `min_dist` m), or every `max_gap` m in a straight line.
  * hazard beacon  when the vision node reports a new hazard (radiation,
                   fire, gas): dropped where the robot stands, and linked like
                   any other beacon.
  * no duplicates  a trail beacon is not dropped within `dedup` m of an older
                   beacon in plain sight (the robot is driving back along its
                   own trail). The robot "passes" that older beacon instead,
                   and the next new beacon links to it, so the tree takes the
                   short way round. `line_free(a, b)` (from the SLAM map) says
                   whether the straight line a-b is clear: a beacon behind a
                   wall never counts.

Beacons fall out of the trapdoor under the magazine, `magazine_x` m behind the
robot centre: that point is the beacon's position.
"""
import math

MAGAZINE_X = -0.12


class BeaconTrail:
    def __init__(self, turn_deg=35.0, min_dist=0.8, max_gap=2.5, dedup=1.2, pass_radius=0.5,
                 magazine=120, exit_beacon=True, magazine_x=MAGAZINE_X, line_free=None):
        self.line_free = line_free
        self.turn = math.radians(turn_deg)
        self.min_dist = min_dist
        self.max_gap = max_gap
        self.dedup = dedup
        self.pass_radius = pass_radius
        self.magazine = magazine
        self.exit_beacon = exit_beacon
        self.mag_x = magazine_x
        self.beacons = []          # every beacon dropped: dict (beacon_id, event_type, x, y, theta, link_id)
        self.next_id = 0
        self.last_link = None      # id of the beacon the robot dropped or passed last
        self.last_drop = None      # id of the beacon the robot dropped last
        self.ref = None            # (x, y, theta) of the last trail decision

    # ------------------------------------------------------------ helpers
    def drop_point(self, x, y, th):
        return x + self.mag_x * math.cos(th), y + self.mag_x * math.sin(th)

    def remaining(self):
        return self.magazine - len(self.beacons)

    def nearest(self, x, y, exclude=None, radius=None, clear=False):
        """(distance, beacon) of the nearest beacon, optionally within
        `radius` and in plain sight (clear straight line)."""
        cands = []
        for b in self.beacons:
            if b['beacon_id'] == exclude:
                continue
            d = math.hypot(b['x'] - x, b['y'] - y)
            if radius is None or d < radius:
                cands.append((d, b['beacon_id'], b))
        for d, _, b in sorted(cands):
            if not clear or self.line_free is None or self.line_free((x, y), (b['x'], b['y'])):
                return (d, b)
        return None

    def _new(self, event_type, x, y, th, **extra):
        if self.remaining() <= 0:
            return None
        b = {'beacon_id': self.next_id, 'event_type': event_type,
             'x': round(x, 3), 'y': round(y, 3), 'theta': round(th, 3),
             'link_id': self.last_link}
        b.update(extra)
        self.next_id += 1
        self.beacons.append(b)
        self.last_link = self.last_drop = b['beacon_id']
        return b

    # ------------------------------------------------------------ inputs
    def update(self, x, y, th):
        """Robot pose (map frame), called a few times per second.
        Returns the list of beacons to drop now (0 or 1)."""
        bx, by = self.drop_point(x, y, th)
        if self.ref is None:
            self.ref = (x, y, th)
            if self.exit_beacon:
                b = self._new('exit', bx, by, th)
                return [b] if b else []
            return []
        # driving past an older beacon: the way out now continues from it
        near = self.nearest(bx, by)
        if near and near[0] < self.pass_radius and near[1]['beacon_id'] != self.last_link:
            self.last_link = near[1]['beacon_id']
        rx, ry, rth = self.ref
        dist = math.hypot(x - rx, y - ry)
        dth = abs(math.atan2(math.sin(th - rth), math.cos(th - rth)))
        if not ((dth > self.turn and dist > self.min_dist) or dist > self.max_gap):
            return []
        self.ref = (x, y, th)
        # an older beacon close by? (the robot is retracing its trail) - pass it
        # instead of dropping. The beacon just dropped on this same stretch does
        # not count, so a corner beacon 0.8 m after it is still dropped.
        exclude = self.last_link if self.last_link == self.last_drop else None
        near = self.nearest(bx, by, exclude=exclude, radius=self.dedup, clear=True)
        if near:
            self.last_link = near[1]['beacon_id']
            return []
        b = self._new('waypoint', bx, by, th)
        return [b] if b else []

    def ensure_exit(self, x, y, th):
        """The EXIT beacon must be the first one (the root of the tree). An event reported before the
        first trail check (a seam right at the entrance, say) drops it first: returns [exit] or []."""
        if self.ref is None and self.exit_beacon:
            return self.update(x, y, th)
        return []

    def add_event(self, event_type, x, y, th, **extra):
        """A hazard to mark: dropped where the robot stands now. Returns the
        beacon dict, or None when the magazine is empty."""
        bx, by = self.drop_point(x, y, th)
        return self._new(event_type, bx, by, th, **extra)
