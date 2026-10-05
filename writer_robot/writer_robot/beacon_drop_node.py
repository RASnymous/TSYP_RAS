#!/usr/bin/env python3
"""
Beacon Drop Node - Writer Robot (TSYP14 Living Map)
Stack: ROS 2 Lyrical + Gazebo Jetty (gz-sim)

Deposits physical LoRa beacons that the Executor robot later follows. The
decisions are in writer_robot/beacon_logic.py (no ROS inside, also used by the
test simulators):

  0. EXIT beacon  - at the start pose: the way in, and the way out.
  1. TRAIL beacon - after a heading change of more than `turn_threshold_deg`,
     or every `max_trail_gap_m` m in a straight line.
  2. HAZARD beacon - when the vision node reports a radiation/fire/gas event.
     The beacon drops where the robot stands (it never goes near the hazard);
     its message carries the hazard's own map position (event_x, event_y).

Every beacon LINKS to the last beacon the robot dropped or drove past
(`link_id`). This becomes its LMB2 `next`, "the way out", so the beacons
form a tree rooted at the EXIT that the Executor can follow in both
directions. No trail beacon is dropped within `trail_dedup_m` of an older
beacon in plain sight (the robot is retracing its trail). Plain sight is
checked on the SLAM /map.

For each drop it runs the real dispenser sequence:
    OPEN servo (/trapdoor_cmd) -> SPAWN a beacon model in Gazebo
    (ros_gz_sim create) -> CLOSE servo -> publish /writer/beacon_dropped.
The beacon model is static and visual-only (no collision), and lower than the
LiDAR, so a dropped beacon can never block, push or confuse a robot.

All positions use the SLAM `map` frame (TF map->base_footprint); /odom is only
a fallback if TF is not ready yet. /writer/beacon_markers shows every beacon
in RViz (green exit, blue trail, magenta radiation, orange fire, yellow gas).
"""
import json
import math
import os
import shutil
import subprocess
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from writer_robot.beacon_logic import BeaconTrail

DEFAULT_TTL = 20

VALVE_CLOSED = 0.0
VALVE_OPEN = -1.2
T_OPEN, T_DROP, T_CLOSE = 0.4, 0.6, 0.4

DROP_Z = 0.0               # the beacon model's origin is on the ground

COLORS = {                  # RGBA for RViz markers
    'exit':      (0.1, 0.9, 0.3, 1.0),   # green
    'waypoint':  (0.1, 0.4, 1.0, 1.0),   # blue trail
    'radiation': (0.9, 0.0, 0.9, 1.0),   # magenta
    'fire':      (1.0, 0.35, 0.0, 1.0),  # orange
    'gas':       (1.0, 0.85, 0.0, 1.0),  # yellow
    'victim':    (0.95, 0.3, 0.6, 1.0),  # pink
    'structural': (0.85, 0.55, 0.1, 1.0),  # amber: unstable roof
    'phosphate': (0.55, 0.45, 0.3, 1.0),  # ochre: phosphate seam
    'gold':      (1.0, 0.8, 0.2, 1.0),   # gold
    'gemstone':  (0.1, 0.9, 0.75, 1.0),  # teal
    'searched':  (0.55, 0.95, 0.55, 1.0),  # light green: area searched (victim search)
}


class BeaconDropNode(Node):
    def __init__(self):
        super().__init__('beacon_drop_node')

        self.declare_parameter('magazine_capacity', 120)
        self.declare_parameter('world_name', 'contaminated_zone')
        self.declare_parameter('use_gz_spawn', True)
        self.declare_parameter('trail_mode', True)    # trail beacons (exit + hazards always)
        self.declare_parameter('exit_beacon', True)
        self.declare_parameter('turn_threshold_deg', 35.0)
        self.declare_parameter('min_trail_dist_m', 0.8)
        self.declare_parameter('max_trail_gap_m', 2.5)
        self.declare_parameter('trail_dedup_m', 1.2)
        self.declare_parameter('pass_radius_m', 0.5)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.world_name = g('world_name')
        self.use_spawn = g('use_gz_spawn')
        self.trail_mode = g('trail_mode')
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.trail = BeaconTrail(turn_deg=float(g('turn_threshold_deg')), min_dist=float(g('min_trail_dist_m')),
                                 max_gap=float(g('max_trail_gap_m')), dedup=float(g('trail_dedup_m')),
                                 pass_radius=float(g('pass_radius_m')), magazine=int(g('magazine_capacity')),
                                 exit_beacon=bool(g('exit_beacon')), line_free=self.line_free)

        # TF: the accurate, drift-corrected robot pose (map -> base_footprint)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.have_odom = False
        self.map = None             # (int8 array, res, ox, oy) - plain-sight checks

        self.queue = deque()
        self.active = None
        self.state = 'IDLE'
        self.t_state = self.get_clock().now()
        self.shown = []             # beacons drawn in RViz: (id, x, y, event_type, link_id)
        # Gazebo model names must be unique: a second run in the same Gazebo
        # (node restarted) would otherwise fail to spawn beacon_0, beacon_1...
        self.run_tag = f'{int(time.time()) % 100000:05d}'

        self.sdf_path = os.path.join(get_package_share_directory('writer_robot'),
                                     'models', 'lora_beacon', 'model.sdf')
        if self.use_spawn and (shutil.which('ros2') is None or not os.path.exists(self.sdf_path)):
            self.get_logger().warning('gz spawn unavailable; physical drop off (messages + markers still work).')
            self.use_spawn = False

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/writer/events', self.on_event, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.servo_pub = self.create_publisher(Float64, '/trapdoor_cmd', 10)
        # transient local: a node that starts a little later (mission_recorder,
        # beacon_record_node...) still receives every beacon, the EXIT included
        drop_qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.beacon_pub = self.create_publisher(String, '/writer/beacon_dropped', drop_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/writer/beacon_markers', 10)

        self._set_valve(VALVE_CLOSED)
        self.create_timer(0.05, self.tick)          # 20 Hz dispenser
        self.create_timer(0.2, self.trail_check)    # 5 Hz trail decisions
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'Beacon Drop Node ready. Magazine={self.trail.magazine}, '
                               f'trail_mode={self.trail_mode}, gz_spawn={self.use_spawn}, frame={self.map_frame}')

    # ----------------------------------------------------- pose / map
    def _pose(self):
        """Robot pose in the map frame (x, y, theta). Falls back to /odom."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            q = t.transform.rotation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            return (t.transform.translation.x, t.transform.translation.y, math.atan2(siny, cosy))
        except tf2_ros.TransformException:
            if self.have_odom:
                return (self.odom['x'], self.odom['y'], self.odom['theta'])
            return None

    def on_odom(self, msg: Odometry):
        self.odom['x'] = msg.pose.pose.position.x
        self.odom['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odom['theta'] = math.atan2(siny, cosy)
        self.have_odom = True

    def on_map(self, msg: OccupancyGrid):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.map = (data, info.resolution, info.origin.position.x, info.origin.position.y)

    def line_free(self, a, b):
        """No occupied SLAM cell on the straight line a-b (True without a map)."""
        if self.map is None:
            return True
        data, res, ox, oy = self.map
        h, w = data.shape
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res * 0.5)) + 2
        for s in np.linspace(0.0, 1.0, n):
            j = int((a[0] + (b[0] - a[0]) * s - ox) / res)
            i = int((a[1] + (b[1] - a[1]) * s - oy) / res)
            if 0 <= i < h and 0 <= j < w and data[i, j] >= 50:
                return False
        return True

    # ----------------------------------------------------- decisions
    def on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)          # hazard event from vision
        except (json.JSONDecodeError, TypeError):
            ev = None
        if not isinstance(ev, dict):
            self.get_logger().error('Bad event message, ignored.')
            return
        et = str(ev.get('event_type', 'waypoint'))
        p = self._pose()                       # the beacon drops where the robot is NOW
        if p is None and ev.get('x') is not None and ev.get('y') is not None:
            try:
                p = (float(ev['x']), float(ev['y']), float(ev.get('theta') or 0.0))
            except (TypeError, ValueError):
                p = None
        if p is None:
            self.get_logger().warning(f'{et} event but no robot pose yet - no beacon')
            return
        extra = {k: ev[k] for k in ('event_x', 'event_y', 'range_m', 'bearing_rad', 'value', 'grade', 'detail')
                 if ev.get(k) is not None}
        self.queue.extend(self.trail.ensure_exit(p[0], p[1], p[2]))     # the EXIT always comes first
        b = self.trail.add_event(et, p[0], p[1], p[2], **extra)
        if b is None:
            self.get_logger().warning(f'Magazine EMPTY - cannot drop {et} beacon.')
            return
        self.queue.append(b)

    def trail_check(self):
        p = self._pose()
        if p is None:
            return
        if not self.trail_mode and self.trail.ref is not None:
            return                      # exit beacon dropped, no trail beacons wanted
        for b in self.trail.update(*p):
            if b['event_type'] == 'exit' or self.trail_mode:
                self.queue.append(b)

    # ----------------------------------------------------- servo helpers
    def _set_valve(self, angle):
        cmd = Float64()
        cmd.data = float(angle)
        self.servo_pub.publish(cmd)

    def _elapsed(self):
        e = (self.get_clock().now() - self.t_state).nanoseconds * 1e-9
        if e < 0.0:                    # sim time went backwards (world reset)
            self.t_state = self.get_clock().now()
            e = 0.0
        return e

    def _to(self, state):
        self.state = state
        self.t_state = self.get_clock().now()

    # ----------------------------------------------------- dispenser FSM
    def tick(self):
        if self.state == 'IDLE':
            if not self.queue:
                return
            self.active = self.queue.popleft()
            self._set_valve(VALVE_OPEN)
            self._to('OPENING')
        elif self.state == 'OPENING':
            if self._elapsed() >= T_OPEN:
                self._spawn_beacon(self.active)
                self._show(self.active)        # RViz at the same moment as Gazebo
                self._to('DROPPING')
        elif self.state == 'DROPPING':
            if self._elapsed() >= T_DROP:
                self._set_valve(VALVE_CLOSED)
                self._publish_beacon(self.active)
                self._to('CLOSING')
        elif self.state == 'CLOSING':
            if self._elapsed() >= T_CLOSE:
                self._to('IDLE')

    # ----------------------------------------------------- actions
    def _spawn_beacon(self, b):
        if not self.use_spawn:
            return
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-world', self.world_name, '-file', self.sdf_path,
               '-name', f'beacon_{self.run_tag}_{b["beacon_id"]}',
               '-x', f'{float(b["x"]):.3f}', '-y', f'{float(b["y"]):.3f}', '-z', f'{DROP_Z:.3f}']
        try:
            # output kept in /tmp/beacon_spawn.log (a failed spawn is visible there)
            with open('/tmp/beacon_spawn.log', 'a') as log:
                subprocess.Popen(cmd, stdout=log, stderr=log)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'gz spawn failed: {e}')

    def _publish_beacon(self, b):
        et = b.get('event_type', 'waypoint')
        beacon = {
            'beacon_id': b['beacon_id'],
            'event_type': et,
            'value': b.get('value', 0.0),
            'x': b['x'], 'y': b['y'], 'theta': b.get('theta'),   # where the beacon lies
            'link_id': b.get('link_id'),                         # its way out (LMB2 `next`)
            'timestamp': time.time(),
            'ttl': DEFAULT_TTL,
            'magazine_remaining': self.trail.remaining(),
        }
        for k in ('event_x', 'event_y', 'range_m', 'bearing_rad', 'grade', 'detail'):
            if b.get(k) is not None:
                beacon[k] = b[k]
        out = String()
        out.data = json.dumps(beacon)
        self.beacon_pub.publish(out)
        link = beacon['link_id']
        self.get_logger().info(
            f"Beacon #{beacon['beacon_id']} deposited ({et}) at ({b['x']:.2f}, {b['y']:.2f})"
            + (f', way out -> #{link}' if link is not None else ' (root: the way out)')
            + f', {beacon["magazine_remaining"]} left')

    # ----------------------------------------------------- RViz markers
    def _show(self, b):
        self.shown.append((int(b['beacon_id']), float(b['x']), float(b['y']),
                           b.get('event_type', 'waypoint'), b.get('link_id')))
        self.publish_markers()

    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.map_frame     # stamp 0 = "latest": no TF-time issues
        m.ns = ns
        m.id = int(mid)
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_markers(self):
        """Every beacon as a post that sticks out ABOVE the robot (the beacon
        itself drops under the robot, where a flat marker would be hidden),
        with its id, plus the links to the way out (LMB2 next)."""
        if not self.shown:
            return
        ma = MarkerArray()
        pos = {bid: (x, y) for bid, x, y, _, _ in self.shown}
        links = self._marker('beacon_links', 0, Marker.LINE_LIST)
        links.scale.x = 0.04
        links.color.r, links.color.g, links.color.b, links.color.a = 0.3, 0.8, 1.0, 0.9
        for bid, x, y, et, link in self.shown:
            r, g, b_, a = COLORS.get(et, COLORS['waypoint'])
            big = et != 'waypoint'
            post = self._marker('beacon_posts', bid, Marker.CYLINDER)
            post.pose.position.x, post.pose.position.y, post.pose.position.z = x, y, 0.25
            post.scale.x = post.scale.y = 0.05
            post.scale.z = 0.5
            post.color.r, post.color.g, post.color.b, post.color.a = r, g, b_, 0.9
            head = self._marker('beacons', bid, Marker.SPHERE)
            head.pose.position.x, head.pose.position.y, head.pose.position.z = x, y, 0.55
            head.scale.x = head.scale.y = head.scale.z = 0.3 if big else 0.2
            head.color.r, head.color.g, head.color.b, head.color.a = r, g, b_, a
            label = self._marker('beacon_ids', bid, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.85
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = ('EXIT ' if et == 'exit' else '') + f'#{bid}' + (f' {et}' if big and et != 'exit' else '')
            ma.markers.extend([post, head, label])
            if link is not None and link in pos:
                links.points.append(Point(x=x, y=y, z=0.55))
                links.points.append(Point(x=pos[link][0], y=pos[link][1], z=0.55))
        if links.points:
            ma.markers.append(links)
        self.marker_pub.publish(ma)


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BeaconDropNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
