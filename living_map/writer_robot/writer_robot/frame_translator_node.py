#!/usr/bin/env python3
"""
Frame Translator Node - Outside Network Area stand-in (TSYP14 Living Map)

Subscribes to /writer/beacon_dropped (local robot-frame coordinates) and
converts them into real-world GPS coordinates using a simple local
tangent-plane (equirectangular) approximation anchored at a fixed
last-known-GPS point (captured before the robot entered the GPS-denied
zone). Publishes the translated message on /outside_network/beacon_gps,
which is what the FastAPI backend / MQTT bridge should subscribe to.

NOTE: this node currently runs standalone for local testing; in the full
system it belongs conceptually on the Outside Network gateway, not on the
robot itself.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from writer_robot.mission_log import GeoFrame

# --- Anchor point: replace with the real last-GPS-fix captured just
# before entering the disconnected zone. Placeholder = ENSIT, Tunis area.
ANCHOR_LAT = 36.8065
ANCHOR_LON = 10.1815
EARTH_RADIUS_M = 6378137.0


class FrameTranslatorNode(Node):
    def __init__(self):
        super().__init__('frame_translator_node')
        self.declare_parameter('anchor_lat', ANCHOR_LAT)
        self.declare_parameter('anchor_lon', ANCHOR_LON)
        self.declare_parameter('map_yaw_deg', 0.0)   # compass direction of the map +x axis (0 = east)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        # the same conversion as mission_recorder (map x/y -> east/north -> lat/lon)
        self.geo = GeoFrame(g('anchor_lat'), g('anchor_lon'), g('map_yaw_deg'))
        self.beacon_sub = self.create_subscription(
            String, '/writer/beacon_dropped', self.beacon_callback, 50)
        self.gps_pub = self.create_publisher(String, '/outside_network/beacon_gps', 10)
        self.get_logger().info(
            f'Frame Translator Node started. Anchor=({self.geo.lat0},{self.geo.lon0}), '
            f'map yaw {g("map_yaw_deg")} deg')

    def beacon_callback(self, msg: String):
        try:
            beacon = json.loads(msg.data)
            x, y = float(beacon['x']), float(beacon['y'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('bad beacon message ignored')
            return
        lat, lon = self.geo.to_latlon(x, y)
        translated = dict(beacon)
        translated['lat'] = round(lat, 7)
        translated['lon'] = round(lon, 7)
        if beacon.get('event_x') is not None and beacon.get('event_y') is not None:
            try:
                elat, elon = self.geo.to_latlon(float(beacon['event_x']), float(beacon['event_y']))
                translated['event_lat'], translated['event_lon'] = round(elat, 7), round(elon, 7)
            except (TypeError, ValueError):
                pass
        out = String()
        out.data = json.dumps(translated)
        self.gps_pub.publish(out)
        self.get_logger().info(
            f"Translated beacon #{beacon.get('beacon_id')} -> GPS ({lat:.6f}, {lon:.6f})")


def main(args=None):
    rclpy.init(args=args)
    node = FrameTranslatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
