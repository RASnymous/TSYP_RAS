#!/usr/bin/env python3
"""
Executor Node - Executor Robot (TSYP14 Living Map)
Stack: ROS 2 Lyrical + Gazebo Jetty, slam_toolbox. No Nav2.

Follows the Writer's beacons to every hazard, treats each one from a safe
standoff, and returns to the exit. The decisions are made by
writer_robot/executor_core.py (no ROS inside, tested in tools/sim_mission.py);
this node only connects it to ROS:

  in   /executor/lora_rx   std_msgs/String JSON  decoded LMB2 records (beacon_radio_sim,
                                                  or a real LoRa gateway bridge)
       /executor/hazard    std_msgs/String JSON  the Executor's own camera (vision node)
       /executor/briefing  std_msgs/String JSON  v9: a Command Post briefing, signature already checked
                                                  by ona_link_node {mission_id, seq, targets, return_to_exit,
                                                  abort, in_order}
       /map, /scan, TF map -> base_footprint
  out  /cmd_vel                                   10 Hz
       /executor/plan           nav_msgs/Path    current path
       /executor/route          nav_msgs/Path    the beacons still to visit
       /executor/markers        MarkerArray      beacon tree, targets, keep-out zones
       /executor/status         std_msgs/String  phase + what it is doing
       /executor/mission_events std_msgs/String  JSON per treated hazard
       /executor/telemetry      std_msgs/String  v9, 1 Hz JSON: phase, mission_id, ack, done/total,
                                                  last beacon (ona_link_node puts it in the ROBOT frames)

Each treated hazard gets a green disc on the floor in Gazebo (models/treated_marker).

Run (sim): ros2 run writer_robot executor_node --ros-args -p use_sim_time:=true
"""
import dataclasses
import json
import math
import os
import shutil
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from writer_robot.executor_core import ExecutorCore, ExecParams
from writer_robot.tf_pose import lookup_pose
from writer_robot.mission_log import GeoFrame

KIND_RGB = {'EXIT': (0.1, 0.9, 0.3), 'WAYPOINT': (0.2, 0.5, 1.0), 'RADIATION': (0.9, 0.0, 0.9),
            'THERMAL': (1.0, 0.25, 0.0), 'GAS': (1.0, 0.85, 0.0), 'VICTIM': (0.0, 0.9, 0.9)}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('executor_node')
        params = ExecParams()
        for f in dataclasses.fields(ExecParams):
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
        self.declare_parameter('start_delay_s', 3.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('use_gz_spawn', True)
        self.declare_parameter('world_name', 'contaminated_zone')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')
        self.odom_frame = g('odom_frame')
        self.geo_from_anchor = False
        self.start_delay = float(g('start_delay_s'))
        self.world_name = g('world_name')
        self.spawn = bool(g('use_gz_spawn')) and shutil.which('ros2') is not None
        try:
            from ament_index_python.packages import get_package_share_directory
            self.marker_sdf = os.path.join(get_package_share_directory('writer_robot'),
                                           'models', 'treated_marker', 'model.sdf')
        except Exception:  # noqa: BLE001
            self.marker_sdf = ''
        if not os.path.exists(self.marker_sdf):
            self.spawn = False

        self.core = ExecutorCore(params, log=lambda m: self.get_logger().info(m), wall_clock=time.time)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/executor/lora_rx', self.on_record, 100)
        self.create_subscription(String, '/executor/hazard', self.on_hazard, 20)
        # latched: a briefing published before this node started is still delivered
        brief_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/executor/briefing', self.on_briefing, brief_qos)
        self.telemetry_pub = self.create_publisher(String, '/executor/telemetry', 5)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, '/executor/plan', 5)
        self.route_pub = self.create_publisher(Path, '/executor/route', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/executor/markers', 5)
        self.status_pub = self.create_publisher(String, '/executor/status', 5)
        self.event_pub = self.create_publisher(String, '/executor/mission_events', 20)
        self.t_start = None
        self.last_scan_t = None
        self.n_events = 0
        self.tick_n = 0
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Executor ready: waiting for beacon records on /executor/lora_rx, /map, /scan and TF.')

    # ------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg):
        info = msg.info
        if info.width == 0 or info.height == 0 or abs(yaw_of(info.origin.orientation)) > 1e-3:
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.core.set_map(data, info.resolution, info.origin.position.x, info.origin.position.y, self._now())

    def on_scan(self, msg):
        pose = self._pose(rclpy.time.Time.from_msg(msg.header.stamp))
        self.core.set_scan(np.asarray(msg.ranges, dtype=float), msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max, pose=pose)
        self.last_scan_t = self._now()

    def on_record(self, msg):
        try:
            r = json.loads(msg.data)
            int(r['beacon_id'])
            float(r['x'])
            float(r['y'])
            str(r['kind'])
            nx = r.get('next')
            if nx is not None:
                int(nx['id'])
                float(nx.get('dist_m') or 0.0)
                if nx.get('bearing_deg') is not None:
                    float(nx['bearing_deg'])
        except (ValueError, KeyError, TypeError, AttributeError):
            self.get_logger().warning('malformed beacon record ignored', throttle_duration_sec=5.0)
            return
        if not self.geo_from_anchor and isinstance(r.get('anchor'), dict):
            # the anchor (and map_yaw_deg) of the mission: bearings depend on it
            a = r['anchor']
            self.core.geo = GeoFrame(a.get('lat', 36.8065), a.get('lon', 10.1815), a.get('map_yaw_deg', 0.0))
            self.geo_from_anchor = True
        if self.core.geo is None:
            self.core.geo = GeoFrame()      # until a record with an anchor arrives
        self.core.add_record(r, self._now())

    def on_briefing(self, msg):
        try:
            m = json.loads(msg.data)
            int(m['mission_id'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('malformed briefing ignored')
            return
        if self.core.set_briefing(m, self._now()):
            self.get_logger().info(f'briefing {m["mission_id"]} accepted: '
                                   + ('ABORT' if m.get('abort') else ' -> '.join(f'#{t}' for t in m.get('targets', []))))

    def on_hazard(self, msg):
        try:
            ev = json.loads(msg.data)
            kind, x, y = str(ev['type']), float(ev['x']), float(ev['y'])
        except (ValueError, KeyError, TypeError):
            return
        self.core.on_sighting(kind, x, y, self._now())

    def _pose(self, stamp=None):
        """Robot pose in the map frame (at `stamp` if given, else the freshest)."""
        return lookup_pose(self.tf_buffer, self.map_frame, self.base_frame, self.odom_frame, stamp)

    # ------------------------------------------------------------ loop
    def tick(self):
        now = self._now()
        if self.t_start is None:
            self.t_start = now
        cmd = Twist()
        pose = self._pose()
        if pose is None or now - self.t_start < self.start_delay:
            self._status('waiting for TF map -> base_footprint' if pose is None else 'starting...')
            self.cmd_pub.publish(cmd)
            return
        if self.last_scan_t is None or now - self.last_scan_t > 1.5:
            self._status('no /scan - stopped')
            self.cmd_pub.publish(cmd)
            return
        self.core.set_pose(*pose)
        v, w = self.core.step(now)
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)
        self._status(f'{self.core.phase}: {self.core.status}')
        while self.n_events < len(self.core.mission_events):
            ev = self.core.mission_events[self.n_events]
            self.n_events += 1
            out = String()
            out.data = json.dumps(ev)
            self.event_pub.publish(out)
            self._spawn_treated(ev)
        self.tick_n += 1
        if self.tick_n % 10 == 0:
            tm = String()
            tm.data = json.dumps({**self.core.progress(), 'status': self.core.status, 'state': self.core.state})
            self.telemetry_pub.publish(tm)
        if self.tick_n % 5 == 0:
            self._publish_paths()
        if self.tick_n % 10 == 0:
            self._publish_markers()

    def _status(self, text):
        m = String()
        m.data = text
        self.status_pub.publish(m)

    def _spawn_treated(self, ev):
        if not self.spawn:
            return
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world_name, '-file', self.marker_sdf,
               '-name', f'treated_{ev["id"]}', '-x', f'{ev["x"]:.3f}', '-y', f'{ev["y"]:.3f}', '-z', '0.000']
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'gz spawn failed: {e}')

    # ------------------------------------------------------------ visualisation
    def _path_msg(self, pts):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _publish_paths(self):
        c = self.core
        here = [c.pose[:2]] if c.pose else []
        self.plan_pub.publish(self._path_msg(here + list(c.path) if c.path else []))
        rest = [(q[0], q[1]) for q in c.route[c.route_i:]] if c.route and c.phase not in ('DONE', 'WAIT') else []
        self.route_pub.publish(self._path_msg(here + rest if rest else []))

    def _marker(self, ns, mid, mtype, stamp):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, int(mid), mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self):
        c = self.core
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        done = {d['id'] for d in c.done_targets}
        skipped = {s_[0] for s_ in c.skipped}
        links = self._marker('links', 0, Marker.LINE_LIST, stamp)
        links.scale.x = 0.04
        links.color.r, links.color.g, links.color.b, links.color.a = 0.4, 0.6, 1.0, 0.9
        zones = []                                   # (x, y, rgb, treated, label or None)
        for n in c.nodes.values():
            r, g, b = KIND_RGB.get(n.kind, (0.7, 0.7, 0.7))
            if n.drop is not None:
                # a post sticking out above the robot (the beacon lies on the floor,
                # where a flat marker would be hidden inside the robot model)
                post = self._marker('beacon_posts', n.id, Marker.CYLINDER, stamp)
                post.pose.position.x, post.pose.position.y, post.pose.position.z = float(n.drop[0]), float(n.drop[1]), 0.25
                post.scale.x = post.scale.y = 0.05
                post.scale.z = 0.5
                post.color.r, post.color.g, post.color.b, post.color.a = r, g, b, 0.9
                m = self._marker('beacons', n.id, Marker.SPHERE, stamp)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(n.drop[0]), float(n.drop[1]), 0.55
                s_ = 0.2 if n.kind == 'WAYPOINT' else 0.3
                m.scale.x = m.scale.y = m.scale.z = s_
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
                ma.markers.extend([post, m])
                p = c.nodes.get(n.parent) if n.parent is not None else None
                if p is not None and p.drop is not None:
                    links.points.append(Point(x=float(n.drop[0]), y=float(n.drop[1]), z=0.55))
                    links.points.append(Point(x=float(p.drop[0]), y=float(p.drop[1]), z=0.55))
            if n.kind not in ('WAYPOINT', 'EXIT'):
                ok = n.id in done
                state = 'TREATED' if ok else ('skipped' if n.id in skipped or n.id not in c.targets else 'target')
                zones.append((n.x, n.y, (r, g, b), ok, f'{n.kind} #{n.id} {state}'))
        # keep-out zones the Executor added from its own camera (not in any record)
        for h in c.hazards:
            if not any(n.kind not in ('WAYPOINT', 'EXIT') and math.hypot(n.x - h.x, n.y - h.y) < 1.0
                       for n in c.nodes.values()):
                zones.append((h.x, h.y, (1.0, 0.3, 0.3), False, f'{h.kind} (camera)'))
        for k, (x, y, rgb, ok, text) in enumerate(zones):
            z = self._marker('keepout', k, Marker.CYLINDER, stamp)
            z.pose.position.x, z.pose.position.y, z.pose.position.z = float(x), float(y), 0.02
            d = 2.0 * c.p.keepout_radius
            z.scale.x, z.scale.y, z.scale.z = d, d, 0.03
            z.color.r, z.color.g, z.color.b = (0.1, 0.9, 0.3) if ok else rgb
            z.color.a = 0.3
            t = self._marker('label', k, Marker.TEXT_VIEW_FACING, stamp)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(x), float(y), 0.9
            t.scale.z = 0.28
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = text
            ma.markers.extend([z, t])
        if links.points:
            ma.markers.append(links)
        if c.pose is not None:
            t = self._marker('status', 0, Marker.TEXT_VIEW_FACING, stamp)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(c.pose[0]), float(c.pose[1]), 1.1
            t.scale.z = 0.25
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 0.3, 1.0
            t.text = f'EXECUTOR {c.phase}'
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
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = ExecutorNode()
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
