#!/usr/bin/env python3
"""
Frontier Explorer - Writer Robot (TSYP14 Living Map)
Stack: ROS 2 Lyrical + Gazebo Jetty, slam_toolbox. No Nav2.

Drives the Writer robot until the whole reachable map is explored and every
area has been checked by the camera, then brings it back to where it started.
The decisions are made by writer_robot/explorer_core.py (no ROS inside, tested
in tools/sim2d.py); this node only connects it to ROS:

  in   /map               nav_msgs/OccupancyGrid   (slam_toolbox)
       /scan              sensor_msgs/LaserScan
       /writer/hazard     std_msgs/String JSON     (vision node, every sighting)
       /writer/events     std_msgs/String JSON     (vision node, one per hazard)
       TF map -> base_footprint                     (slam_toolbox + odometry)
  out  /cmd_vel           geometry_msgs/Twist      10 Hz
       /writer/plan       nav_msgs/Path            current path
       /writer/explorer_markers  MarkerArray       keep-out zones, goal, status
       /writer/explorer_status   std_msgs/String   state + what it is doing

Hazards (radiation / fire / gas blocks seen by the camera) become 1.2 m
keep-out zones. If one appears in the robot's way it drives back along its
own trail ("RETREAT ... returning the way it came") and carries on exploring
the rest of the map by other routes. It never stops in front of a hazard.

Run (sim):  ros2 run writer_robot frontier_explorer --ros-args -p use_sim_time:=true
"""
import dataclasses
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import tf2_ros
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from writer_robot.explorer_core import ExplorerCore, Params
from writer_robot.tf_pose import lookup_pose

HAZARD_COLORS = {
    'radiation': (0.9, 0.0, 0.9),
    'fire': (1.0, 0.25, 0.0),
    'gas': (1.0, 0.85, 0.0),
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # every explorer parameter can be changed with -p name:=value
        # (dynamic typing: -p stuck_time:=12 works as well as 12.0)
        params = Params()
        for f in dataclasses.fields(Params):
            default = getattr(params, f.name)
            if isinstance(default, tuple):
                continue
            self.declare_parameter(f.name, default, ParameterDescriptor(dynamic_typing=True))
            value = self.get_parameter(f.name).value
            if isinstance(default, bool):
                value = value if isinstance(value, bool) else str(value).lower() in ('1', 'true', 'yes')
            setattr(params, f.name, type(default)(value))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate_hz', 10.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('start_delay_s', 3.0, ParameterDescriptor(dynamic_typing=True))
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.start_delay = float(self.get_parameter('start_delay_s').value)

        self.core = ExplorerCore(params, log=self._core_log)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/writer/hazard', self.on_hazard, 20)
        self.create_subscription(String, '/writer/events', self.on_hazard, 20)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/writer/plan', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/writer/explorer_markers', 5)
        self.status_pub = self.create_publisher(String, '/writer/explorer_status', 5)

        self.t_start = None
        self.last_scan_t = None
        self.last_path_id = None
        self.last_status = ''
        self.tick_n = 0
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info('Frontier explorer ready: waiting for /map, /scan and TF '
                               f'{self.map_frame} -> {self.base_frame}.')

    # ------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _core_log(self, text):
        self.get_logger().info(text)

    # ------------------------------------------------------------ inputs
    def on_map(self, msg: OccupancyGrid):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        if abs(yaw_of(info.origin.orientation)) > 1e-3:
            self.get_logger().warning('map origin is rotated - not supported, map ignored',
                                      throttle_duration_sec=10.0)
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.core.set_map(data, info.resolution, info.origin.position.x, info.origin.position.y, self._now())

    def on_scan(self, msg: LaserScan):
        # robot pose when the scan was taken (fresh hits go into the planning grid)
        pose = self._pose(rclpy.time.Time.from_msg(msg.header.stamp))
        self.core.set_scan(np.asarray(msg.ranges, dtype=float), msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max, pose=pose)
        self.last_scan_t = self._now()

    def on_hazard(self, msg: String):
        try:
            ev = json.loads(msg.data)
            if not isinstance(ev, dict):
                return
            kind = ev.get('type') or ev.get('event_type')
            x = ev.get('x') if 'type' in ev else ev.get('event_x')
            y = ev.get('y') if 'type' in ev else ev.get('event_y')
            if kind in (None, 'waypoint', 'exit', 'phosphate', 'gold', 'gemstone', 'searched') or x is None or y is None:
                return
            self.core.add_hazard(str(kind), float(x), float(y), self._now())
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning(f'bad hazard message ignored: {msg.data[:80]}')

    def _pose(self, stamp=None):
        """Robot pose in the map frame (at `stamp` if given, else the freshest)."""
        return lookup_pose(self.tf_buffer, self.map_frame, self.base_frame, self.odom_frame, stamp)

    # ------------------------------------------------------------ loop
    def tick(self):
        now = self._now()
        if self.t_start is None:
            self.t_start = now
        pose = self._pose()
        cmd = Twist()
        if pose is None:
            self._status('waiting for TF map -> base_footprint (is SLAM running?)')
            self.cmd_pub.publish(cmd)
            return
        if now - self.t_start < self.start_delay:
            self._status('starting...')
            self.cmd_pub.publish(cmd)
            return
        if self.last_scan_t is None or now - self.last_scan_t > 1.5:
            # no fresh LiDAR data: never drive blind
            self._status('no /scan - stopped')
            self.cmd_pub.publish(cmd)
            return
        self.core.set_pose(*pose)
        v, w = self.core.step(now)
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        self._status(f'{self.core.state}: {self.core.status}')
        self._publish_path()
        self.tick_n += 1
        if self.tick_n % 10 == 0:
            self._publish_markers()

    def _status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if text != self.last_status:
            self.last_status = text
            self.get_logger().debug(text)

    def _publish_path(self):
        path = self.core.path
        pid = (id(path), len(path), self.core.state)
        if pid == self.last_path_id and self.tick_n % 10:
            return
        self.last_path_id = pid
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.core.pose is not None and path:
            pts = [self.core.pose[:2]] + list(path)
        else:
            pts = []
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for k, h in enumerate(self.core.hazards):
            r, g, b = HAZARD_COLORS.get(h.kind, (1.0, 0.0, 0.0))
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'keepout'
            m.id = k
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(h.x), float(h.y), 0.02
            m.pose.orientation.w = 1.0
            d = 2.0 * self.core.p.keepout_radius
            m.scale.x, m.scale.y, m.scale.z = d, d, 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.25
            ma.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns = 'keepout_label'
            t.id = k
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(h.x), float(h.y), 0.8
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r, t.color.g, t.color.b, t.color.a = r, g, b, 1.0
            t.text = f'{h.kind.upper()} - keep out'
            ma.markers.append(t)
        if self.core.goal is not None and self.core.state in ('FOLLOW', 'HOME', 'RETREAT', 'LOOK'):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'goal'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                float(self.core.goal[0]), float(self.core.goal[1]), 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            col = {'RETREAT': (1.0, 0.2, 0.2), 'HOME': (0.1, 0.8, 0.3)}.get(self.core.state, (0.1, 0.5, 1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = col[0], col[1], col[2], 0.9
            ma.markers.append(m)
        if self.core.pose is not None:
            t = Marker()
            t.header.frame_id = self.map_frame
            t.header.stamp = stamp
            t.ns = 'status'
            t.id = 0
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = \
                float(self.core.pose[0]), float(self.core.pose[1]), 1.0
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = self.core.state
            ma.markers.append(t)
        self.marker_pub.publish(ma)

    def stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001 - shutting down
            pass


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    # handle Ctrl-C ourselves so the final stop command can still be sent
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
