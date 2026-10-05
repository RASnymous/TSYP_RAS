#!/usr/bin/env python3
"""
Beacon Radio (simulated LoRa receiver) - Executor Robot (TSYP14 Living Map)

Stands in for the LoRa gateway the Executor carries. It "receives" the 44-byte
LMB2 frames of the beacons the Writer dropped (from the Writer's mission file),
checks each one with the mission key (HMAC), decodes it and hands the record
to the Executor. Nothing else from the Writer is used: no map, no positions
other than what is inside the frames.

Every beacon holds every other beacon's record (gossip replication), so the
Executor gets the whole network as soon as it hears the first beacon, the
EXIT beacon at the entrance. The frames are streamed at `rate_hz`, over and
over (a late subscriber still gets everything within one cycle).
Frames that fail the HMAC check are counted and dropped.

  out  /executor/lora_rx   std_msgs/String JSON: the decoded record
       (lmb2 format: beacon_id, seq, kind, gps, severity, confidence, next,
       ts, ttl_s, half_life_s, retracted, stale) + map "x", "y" + "anchor" +
       "frame" (hex) + "rssi"

Parameters: mission_file, key ('demo' or 32 hex), rate_hz.
"""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

from writer_robot.mission_log import load_mission, decode_mission


class BeaconRadioSim(Node):
    def __init__(self):
        super().__init__('beacon_radio_sim')
        self.declare_parameter('mission_file', os.path.expanduser('~/writer_robot_ws/missions/latest.json'))
        self.declare_parameter('key', 'demo')
        self.declare_parameter('rate_hz', 20.0, ParameterDescriptor(dynamic_typing=True))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        path = os.path.expanduser(str(g('mission_file')))
        self.pub = self.create_publisher(String, '/executor/lora_rx', 50)
        self.out = []
        try:
            m = load_mission(path)
            recs, rejected, geo = decode_mission(m, str(g('key')))
        except (OSError, ValueError) as e:
            self.get_logger().error(f'cannot read mission {path}: {e}')
            recs, rejected, geo = [], 0, None
        frames = {int(e['id']): e['frame'] for e in (m.get('beacons', []) if recs else [])}
        for r in recs:
            r['anchor'] = geo.dict()
            r['frame'] = frames.get(int(r['beacon_id']), '')
            r['rssi'] = -95
            self.out.append(json.dumps(r))
        self.i = 0
        self.create_timer(1.0 / max(1.0, float(g('rate_hz'))), self.tick)
        self.get_logger().info(f'LoRa receiver: {len(recs)} beacon frames verified and decoded, '
                               f'{rejected} rejected (bad HMAC) - from {path}')

    def tick(self):
        if not self.out:
            return
        msg = String()
        msg.data = self.out[self.i % len(self.out)]
        self.pub.publish(msg)
        self.i += 1


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BeaconRadioSim()
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
