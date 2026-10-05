#!/usr/bin/env python3
"""
Mission Recorder - Writer Robot (TSYP14 Living Map)

Turns every beacon the Writer drops into the LMB2 record that beacon carries
on the air (44-byte frame, HMAC-sealed with the mission key), and saves the
whole mission to a file after each drop:

    ~/writer_robot_ws/missions/latest.json
    ~/writer_robot_ws/missions/mission_<date>_<time>.json   (same content)

The Executor simulation (run_executor.sh) reads this file: it places the
physical beacons in its Gazebo world and "receives" their 44-byte frames as
it would over LoRa.

  in   /writer/beacon_dropped   std_msgs/String JSON (beacon_drop_node)
       /odom                    distance driven (position error model)
  out  /writer/mission_record   std_msgs/String {"record": {...}, "frame": hex}

Parameters: mission_dir, anchor_lat, anchor_lon, map_yaw_deg, key ('demo' or
32 hex), net_id, err0_m, err_per_m, vision_confidence.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor

from writer_robot.beacon_logic import MAGAZINE_X
from writer_robot.mission_log import GeoFrame, MissionLog


class MissionRecorder(Node):
    def __init__(self):
        super().__init__('mission_recorder')
        num = ParameterDescriptor(dynamic_typing=True)     # 90 or 90.0 both accepted
        p = lambda name, value: self.declare_parameter(  # noqa: E731
            name, value, num if isinstance(value, (int, float)) else ParameterDescriptor())
        p('mission_dir', os.path.join(os.path.expanduser('~'), 'writer_robot_ws', 'missions'))
        p('anchor_lat', 36.8065)
        p('anchor_lon', 10.1815)
        p('map_yaw_deg', 0.0)
        p('key', 'demo')
        p('net_id', 0x2A)
        p('err0_m', 0.3)
        p('err_per_m', 0.02)
        p('vision_confidence', 0.9)
        p('world', '')                 # the Gazebo world file (the Executor uses the same)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        # record time stamps follow SIM time (anchored to the wall clock at the
        # first reading): the Executor ages hazards with the same clock, and
        # Gazebo in a VM runs slower than real time
        self.t0_wall, self.t0_sim = None, None
        self.dir = os.path.expanduser(str(g('mission_dir')))
        self.log_ = MissionLog(GeoFrame(float(g('anchor_lat')), float(g('anchor_lon')), float(g('map_yaw_deg'))),
                               key=str(g('key')),
                               net_id=int(g('net_id')), err0=float(g('err0_m')), err_per_m=float(g('err_per_m')),
                               vision_conf=float(g('vision_confidence')), clock=self._mission_clock)
        self.log_.world = os.path.basename(str(g('world'))) or 'contaminated_zone.world'
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.files = [os.path.join(self.dir, 'latest.json'), os.path.join(self.dir, f'mission_{stamp}.json')]
        self.path_m = 0.0
        self.last_xy = None
        self.pub = self.create_publisher(String, '/writer/mission_record', 20)
        drop_qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/writer/beacon_dropped', self.on_drop, drop_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.get_logger().info(f'Mission recorder: saving to {self.files[0]} (and {os.path.basename(self.files[1])})')

    def _mission_clock(self):
        sim = self.get_clock().now().nanoseconds * 1e-9
        if sim <= 0.0:
            return time.time()          # no /clock yet
        if self.t0_sim is None:
            self.t0_sim, self.t0_wall = sim, time.time()
        return self.t0_wall + (sim - self.t0_sim)

    def on_odom(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < 1.0:                # ignore teleports / resets
                self.path_m += d
        self.last_xy = (x, y)

    def on_drop(self, msg):
        try:
            b = json.loads(msg.data)
            int(b['beacon_id'])
            float(b['x'])
            float(b['y'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('beacon_dropped without id/x/y - skipped')
            return
        if int(b['beacon_id']) in self.log_.drops:
            return                      # already recorded (message delivered twice)
        if b.get('event_type') == 'exit' and self.log_.home is None:
            th = float(b.get('theta') or 0.0)
            self.log_.home = [round(float(b['x']) - MAGAZINE_X * math.cos(th), 3),
                              round(float(b['y']) - MAGAZINE_X * math.sin(th), 3)]
        e = self.log_.add_drop(b, self.path_m)
        if e is None:
            self.get_logger().error(f'beacon #{b["beacon_id"]}: invalid record - skipped')
            return
        out = String()
        out.data = json.dumps({'record': e['record'], 'frame': e['frame'], 'bytes': len(e['frame']) // 2})
        self.pub.publish(out)
        for f in self.files:
            try:
                self.log_.save(f)
            except OSError as ex:
                self.get_logger().error(f'cannot save {f}: {ex}')
        nxt = e['record'].get('next')
        self.get_logger().info(f"#{e['id']} {e['record']['kind']:<9} frame {e['frame'][:16]}..."
                               + (f" next #{nxt['id']} {nxt['dist_m']} m @ {nxt['bearing_deg']:.0f} deg" if nxt else ' (exit)')
                               + f"  [{len(self.log_.entries)} beacons saved]")


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionRecorder()
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
