#!/usr/bin/env python3
"""
ros_bridge.py - Writer Robot  ->  Command Post dashboard
=========================================================
Listens to the beacon node (/writer/beacon_dropped, JSON String) and POSTs
every beacon to the dashboard in the exact data contract:

    POST /api/beacon  {beacon_id, event_type, value, lat, lon,
                       timestamp, ttl, gateway_id, rssi}

It also sends a gateway heartbeat (POST /api/network-health) every few
seconds so the dashboard's network strip shows the link as online.

Conversions:
  * map-frame x/y (metres from the robot's start) -> lat/lon around an anchor
    (default Tunis 36.8065, 10.1815). x = east, y = north unless you set
    map_yaw_deg (the compass direction the robot faced at start, 0 = east).
  * event_type "fire" -> "thermal" (dashboard naming); "waypoint" = blue trail.

Needs only ROS 2 + Python stdlib (no pip installs).

Run (in a sourced ROS terminal, while the robot demo is running):
    python3 ros_bridge.py --ros-args -p server:=http://<dashboard-ip>:3000
"""
import json
import math
import queue
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

M_PER_DEG_LAT = 111320.0


class CommandPostBridge(Node):
    def __init__(self):
        super().__init__('command_post_bridge')
        self.declare_parameter('server', 'http://localhost:3000')
        self.declare_parameter('anchor_lat', 36.8065)
        self.declare_parameter('anchor_lon', 10.1815)
        self.declare_parameter('map_yaw_deg', 0.0)
        self.declare_parameter('gateway_id', 'gw1')
        self.declare_parameter('rssi', -62)
        self.declare_parameter('send_trail', True)       # forward blue trail beacons too
        self.declare_parameter('heartbeat_s', 4.0)

        self.server = self.get_parameter('server').value.rstrip('/')
        self.lat0 = float(self.get_parameter('anchor_lat').value)
        self.lon0 = float(self.get_parameter('anchor_lon').value)
        self.yaw = math.radians(float(self.get_parameter('map_yaw_deg').value))
        self.gw = self.get_parameter('gateway_id').value
        self.rssi = int(self.get_parameter('rssi').value)
        self.send_trail = bool(self.get_parameter('send_trail').value)

        # HTTP runs on a worker thread so a slow/absent server never blocks ROS
        self.outbox = queue.Queue(maxsize=500)
        threading.Thread(target=self._worker, daemon=True).start()

        self.create_subscription(String, '/writer/beacon_dropped', self._on_beacon, 20)
        self.create_timer(float(self.get_parameter('heartbeat_s').value), self._heartbeat)
        self.get_logger().info(f'Bridge -> {self.server}  (anchor {self.lat0}, {self.lon0}, gw={self.gw})')

    # ---------------------------------------------------------------- convert
    def _to_latlon(self, x, y):
        # rotate map frame so +x points east
        e = x * math.cos(self.yaw) - y * math.sin(self.yaw)
        n = x * math.sin(self.yaw) + y * math.cos(self.yaw)
        lat = self.lat0 + n / M_PER_DEG_LAT
        lon = self.lon0 + e / (M_PER_DEG_LAT * math.cos(math.radians(self.lat0)))
        return lat, lon

    def _on_beacon(self, msg):
        try:
            b = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('bad beacon JSON, skipped')
            return
        et = b.get('event_type', 'waypoint')
        if et == 'waypoint' and not self.send_trail:
            return
        if et == 'fire':
            et = 'thermal'
        x, y = b.get('x'), b.get('y')
        if x is None or y is None:
            return
        lat, lon = self._to_latlon(float(x), float(y))
        payload = {
            'beacon_id': b.get('beacon_id'),
            'event_type': et,
            'value': float(b.get('value') or 0.0),
            'lat': round(lat, 7),
            'lon': round(lon, 7),
            'timestamp': time.time(),
            'ttl': int(b.get('ttl') or 20),
            'gateway_id': self.gw,
            'rssi': self.rssi,
        }
        # hazard location (for the Executor) if the vision node provided it
        if b.get('event_x') is not None and b.get('event_y') is not None:
            elat, elon = self._to_latlon(float(b['event_x']), float(b['event_y']))
            payload['event_lat'] = round(elat, 7)
            payload['event_lon'] = round(elon, 7)
        self._send('/api/beacon', payload)
        self.get_logger().info(f"-> beacon #{payload['beacon_id']} {et} @ {lat:.6f},{lon:.6f}")

    def _heartbeat(self):
        self._send('/api/network-health',
                   {'gateway_id': self.gw, 'status': 'online', 'timestamp': time.time()},
                   quiet=True)

    # ---------------------------------------------------------------- http
    def _send(self, path, body, quiet=False):
        try:
            self.outbox.put_nowait((path, body, quiet))
        except queue.Full:
            pass

    def _worker(self):
        warned = False
        while True:
            path, body, quiet = self.outbox.get()
            data = json.dumps(body).encode()
            req = urllib.request.Request(self.server + path, data=data,
                                         headers={'Content-Type': 'application/json'})
            try:
                urllib.request.urlopen(req, timeout=3).read()
                warned = False
            except Exception as e:  # noqa: BLE001
                if not warned or not quiet:
                    self.get_logger().warning(f'POST {path} failed: {e}')
                    warned = True


def main():
    rclpy.init()
    node = CommandPostBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
