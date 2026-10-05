#!/usr/bin/env python3
"""
Event Detection Node - Writer Robot (TSYP14 Living Map)

Simulates radiation and thermal hazard detection.
Real hardware later: swap _read_radiation()/_read_thermal() with actual
Geiger-Mueller tube + MLX90614 driver reads. Kept as simulated triggers per
project safety decision (no real radioactive source).

Publishes JSON strings on /writer/events:
{
  "event_type": "radiation" | "thermal",
  "value": <float>,
  "x": <float>, "y": <float>, "theta": <float>,   # robot pose at detection time
  "timestamp": <float unix time>
}
"""
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class EventDetectionNode(Node):
    def __init__(self):
        super().__init__('event_detection_node')

        self.declare_parameter('radiation_threshold', 0.75)
        self.declare_parameter('thermal_threshold', 0.80)
        self.declare_parameter('check_period_sec', 2.0)

        self.radiation_threshold = self.get_parameter('radiation_threshold').value
        self.thermal_threshold = self.get_parameter('thermal_threshold').value
        period = self.get_parameter('check_period_sec').value

        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.event_pub = self.create_publisher(String, '/writer/events', 10)

        self.timer = self.create_timer(period, self.check_for_events)

        self.get_logger().info('Event Detection Node started (simulated radiation + thermal).')

    def odom_callback(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)

    def _read_radiation(self) -> float:
        # SIMULATED reading in [0,1]. Replace with real Geiger-Mueller
        # pulse-rate normalization on hardware. Occasionally spikes to
        # emulate a hotspot the robot drives near.
        base = random.uniform(0.0, 0.3)
        spike = random.random() < 0.08
        return base + (0.6 if spike else 0.0)

    def _read_thermal(self) -> float:
        # SIMULATED reading in [0,1]. Replace with MLX90614 normalized temp.
        base = random.uniform(0.0, 0.35)
        spike = random.random() < 0.06
        return base + (0.55 if spike else 0.0)

    def check_for_events(self):
        rad = self._read_radiation()
        heat = self._read_thermal()

        if rad >= self.radiation_threshold:
            self._publish_event('radiation', rad)
        if heat >= self.thermal_threshold:
            self._publish_event('thermal', heat)

    def _publish_event(self, event_type: str, value: float):
        msg = String()
        payload = {
            'event_type': event_type,
            'value': round(value, 3),
            'x': round(self.current_pose['x'], 3),
            'y': round(self.current_pose['y'], 3),
            'theta': round(self.current_pose['theta'], 3),
            'timestamp': time.time(),
        }
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)
        self.get_logger().warning(f'EVENT DETECTED -> {payload}')


def main(args=None):
    rclpy.init(args=args)
    node = EventDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
