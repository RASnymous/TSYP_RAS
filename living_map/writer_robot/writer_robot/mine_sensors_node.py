#!/usr/bin/env python3
"""
mine_sensors_node.py - the robot's MINE SENSOR SUITE in the Gazebo mine
(run_explorer.sh mine / run_executor.sh mine). It replaces the colour-block
vision node underground: the dangers of a mine are invisible (gas, radon, a
cracked roof) or in the dark (a fire, a person), and resources must be marked
too. See mine_site.py for the instruments and their models.

    in   /odom          the TRUE pose (Gazebo's OdometryPublisher): where the
                        instruments really are
         TF map->base_footprint   the robot's OWN pose (SLAM): where it puts what
                        it measures on its map
    out  /writer/hazard   every sighting (like the vision node; resources too: the
                        explorer ignores them, the Executor uses them to confirm)
         /writer/events   once per hazard / per 4 m of seam: the beacon node drops
                        a beacon, the recorder writes the record
         /writer/air      1 Hz JSON: CO, NO2, H2S, CH4, O2, radon, gamma

Parameters: site_file (worlds/gafsa_mine.json), world_file (default: from the
site file), rate_hz (5), drop_range_m (4.5), hazard_topic, event_topic,
air_topic, seed. Log: a line every 10 s with the air readings and the
detections so far (/tmp/mine_sensors.log in the run scripts).
"""
import json
import math
import os
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String

from writer_robot.mine_site import MineSite, EventFilter, RESOURCE_TYPES
from writer_robot.tf_pose import lookup_pose


def default_site_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('writer_robot'), 'worlds', 'gafsa_mine.json')
        if os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001 - not in a ROS environment
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'gafsa_mine.json')


class MineSensorsNode(Node):
    def __init__(self):
        super().__init__('mine_sensors')
        P = lambda n, v: self.declare_parameter(n, v, ParameterDescriptor(dynamic_typing=True)).value  # noqa: E731
        site = str(P('site_file', '')) or default_site_file()
        world = str(P('world_file', '')) or None
        self.rate = float(P('rate_hz', 5.0))
        self.map_frame = str(P('map_frame', 'map'))
        self.base_frame = str(P('base_frame', 'base_footprint'))
        hazard_topic = str(P('hazard_topic', '/writer/hazard'))
        event_topic = str(P('event_topic', '/writer/events'))
        air_topic = str(P('air_topic', '/writer/air'))
        self.mine = MineSite(site, world, seed=int(P('seed', 1)))
        self.filter = EventFilter(drop_range=float(P('drop_range_m', 4.5)))
        self.truth = None
        self.counts = {}
        self.last_air = None
        self.radon_info = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.hazard_pub = self.create_publisher(String, hazard_topic, 20)
        self.event_pub = self.create_publisher(String, event_topic, 10)
        self.air_pub = self.create_publisher(String, air_topic, 5)
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(1.0, self.publish_air)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'mine sensors on: {self.mine.site.get("name", site)} - '
                               f'{len(self.mine.items)} items in the site file; multi-gas, radon, gamma, '
                               'thermal camera, roof scanner, wall probe, camera + XRF')

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.truth = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _own_pose(self):
        return lookup_pose(self.tf_buffer, self.map_frame, self.base_frame, 'odom', None)

    def tick(self):
        if self.truth is None:
            return
        own = self._own_pose()
        if own is None:
            return
        x, y, yaw = self.truth
        for d in self.mine.detect(x, y, yaw, est_pose=own):
            self.counts[d.etype] = self.counts.get(d.etype, 0) + 1
            if d.etype == 'searched':
                self._event(d, own)             # an area, not something to avoid: no sighting
                continue
            h = String()
            h.data = json.dumps({'type': d.etype, 'x': round(d.x, 3), 'y': round(d.y, 3), 'range': round(d.range, 2),
                                 'bearing': round(d.bearing, 3), 'stamp': self.get_clock().now().nanoseconds * 1e-9,
                                 'source': 'mine_sensors'})
            self.hazard_pub.publish(h)
            self._event(d, own)

    def _event(self, d, own):
        if not self.filter.new_event(d):
            return
        ev = {'event_type': d.etype, 'value': round(d.value, 3),
              'range_m': round(d.range, 2) if d.etype != 'searched' else 0.0,
              'bearing_rad': round(d.bearing, 3),
              'x': round(own[0], 3), 'y': round(own[1], 3), 'theta': round(own[2], 3),
              'event_x': round(d.x, 3), 'event_y': round(d.y, 3), 'timestamp': time.time(),
              'source': 'mine_sensors', 'detail': d.detail}
        if d.grade is not None:
            ev['grade'] = d.grade
        e = String()
        e.data = json.dumps(ev)
        self.event_pub.publish(e)
        what = ('RESOURCE' if d.etype in RESOURCE_TYPES else
                'VICTIM SEARCH' if d.etype == 'searched' else 'MINE EVENT')
        self.get_logger().warning(f'{what} -> {d.etype} at ({d.x:.2f}, {d.y:.2f}), {d.range:.1f} m: {d.detail}')

    def publish_air(self):
        if self.truth is None:
            return
        r = self.mine.readings(self.truth[0], self.truth[1])
        self.last_air = r
        m = String()
        m.data = json.dumps({k: round(v, 2) for k, v in r.items()})
        self.air_pub.publish(m)
        lim = self.mine.th.get('radon_info_bqm3', 300.0)
        if r['radon_bqm3'] >= lim and not self.radon_info:
            self.get_logger().info(f'radon {r["radon_bqm3"]:.0f} Bq/m3: above the {lim:.0f} Bq/m3 reference level')
        self.radon_info = r['radon_bqm3'] >= lim

    def report(self):
        r = self.last_air
        if r is None:
            self.get_logger().warning('[air] no /odom yet - is the simulation running?')
            return
        seen = ', '.join(f'{k} {v}' for k, v in sorted(self.counts.items())) or 'nothing yet'
        cov = self.mine.coverage()
        if cov:
            seen += ' | victim search: ' + ', '.join(f'{zid} {100 * f:.0f}%' + (f' ({nv}V)' if nv else '')
                                                     for zid, _, f, nv in cov)
        self.get_logger().info(
            f'[air] CO {r["co_ppm"]:.0f} ppm, NO2 {r["no2_ppm"]:.1f} ppm, H2S {r["h2s_ppm"]:.0f} ppm, '
            f'CH4 {r["ch4_lel"]:.0f} %LEL, O2 {r["o2_pct"]:.1f} %, radon {r["radon_bqm3"]:.0f} Bq/m3, '
            f'gamma {r["gamma_usvh"]:.2f} uSv/h | sightings: {seen}')


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MineSensorsNode()
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
