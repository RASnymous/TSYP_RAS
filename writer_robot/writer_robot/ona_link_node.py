#!/usr/bin/env python3
"""
ONA link node - a robot as the Outside Network Area hears it (TSYP14 Living Map, v9)

The robots carry a LoRa radio. Outside the building, three gateways listen and
pass everything to the ONA computer (python -m ona), which votes 2-of-3 on
every frame, measures where the robot is from the three distances (like GPS:
three spheres around three known antennas), and forwards it all to the
Command Post dashboard. This node is the robot's side of that link in the
simulation (writer_robot/ona_radio.py computes what each gateway hears):

  in   TF map -> base_footprint    the robot's OWN pose (its SLAM): what it reports
       /odom (nav_msgs/Odometry)   the TRUE pose (Gazebo's OdometryPublisher): what the
                                   gateways' distances measure
       /executor/telemetry         Executor phase, briefing, progress (executor_node)
       /writer/explorer_status     Writer state (frontier_explorer)
       /writer/mission_record      beacons as the Writer drops them (mission_recorder)
       mission_file parameter      or: every beacon of a saved mission (Executor demo)
  out  UDP -> the ONA              one JSON line per frame and gateway, as the real gateways
                                   print them: robot pings with their distances, beacon
                                   records, gateway heartbeats
       /executor/briefing          a signed MISSION frame from the Command Post, checked
                                   (HMAC) and assembled: the Executor re-targets on it
       /ona_link/status            what the link is doing

Run (sim):
  ros2 run writer_robot ona_link --ros-args -p use_sim_time:=true -p robot:=executor \\
      -p world_file:=/tmp/executor_world.world -p mission_file:=$HOME/.../demo_big.json
  and on the same machine: cd ~/writer_robot_ws/src/writer_robot/ona && \\
      python3 -m ona --config ../config/ona_gazebo_big.json --udp 47100 --command-post http://10.0.2.2:3000
"""
import json
import math
import os
import random
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor

from writer_robot import ona_lmb2 as lmb2
from writer_robot.ona_radio import GatewayRadio, load_world_boxes, load_config, retime_frames, \
    briefing_from_frames
from writer_robot.tf_pose import lookup_pose
from writer_robot.mission_log import GeoFrame

WRITER_PHASES = lmb2.PHASES['WRITER']
EXEC_PHASES = lmb2.PHASES['EXECUTOR']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UdpLink:
    """One socket: lines out to the ONA, its downlink (briefings) back on the same socket."""

    def __init__(self, host, port):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sent = 0
        self.errors = 0

    def send(self, d):
        try:
            self.sock.sendto(json.dumps(d, separators=(',', ':')).encode(), self.addr)
            self.sent += 1
        except OSError:
            self.errors += 1

    def poll(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            for ln in data.decode('utf-8', 'replace').splitlines():
                try:
                    out.append(json.loads(ln))
                except json.JSONDecodeError:
                    pass
        return out


def default_config(world_file):
    try:
        from ament_index_python.packages import get_package_share_directory
        cfg_dir = os.path.join(get_package_share_directory('writer_robot'), 'config')
    except Exception:  # noqa: BLE001
        cfg_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
    name = 'ona_gazebo_retreat.json' if 'retreat' in os.path.basename(world_file or '') else 'ona_gazebo_big.json'
    return os.path.join(cfg_dir, name)


class OnaLinkNode(Node):
    def __init__(self, transport=None):
        super().__init__('ona_link')
        P = lambda n, v: self.declare_parameter(n, v, ParameterDescriptor(dynamic_typing=True)).value  # noqa: E731
        self.robot = str(P('robot', 'executor')).lower()
        rid = int(P('robot_id', 0))
        self.robot_id = rid or (1 if self.robot == 'executor' else 2)
        self.world_file = str(P('world_file', ''))
        cfg_path = str(P('ona_config', '')) or default_config(self.world_file)
        self.mission_file = os.path.expanduser(str(P('mission_file', '')))
        self.live_records = bool(P('live_records', self.robot == 'writer'))
        host, port = str(P('ona_host', '127.0.0.1')), int(P('ona_port', 47100))
        self.ping_period = float(P('ping_period', 2.0))
        self.hb_period = float(P('heartbeat_period', 4.0))
        self.retime = bool(P('retime', True))
        self.advertise_speed = bool(P('advertise_speed', True))
        seed = int(P('seed', 1))
        self.map_frame = str(P('map_frame', 'map'))
        self.base_frame = str(P('base_frame', 'base_footprint'))
        self.odom_frame = str(P('odom_frame', 'odom'))
        truth_topic = str(P('truth_topic', '/odom'))

        self.cfg = load_config(cfg_path)
        self.key, self.net = self.cfg['key'], self.cfg['net_id']
        a = self.cfg.get('anchor') or {}
        self.geo = GeoFrame(a.get('lat', 36.8065), a.get('lon', 10.1815), a.get('map_yaw_deg', 0.0))
        boxes = load_world_boxes(self.world_file) if self.world_file and os.path.exists(self.world_file) else []
        rad = self.cfg.get('radio', {})
        rock = rad.get('rock_db_per_m') if rad.get('medium') == 'rock' else None
        self.radio = GatewayRadio(self.cfg['gateways'], boxes, seed=seed, rock_db_per_m=rock,
                                  tof_needs_los=bool(rad.get('tof_needs_los', False)))
        self.rng = random.Random(seed + 7)
        self.link = transport or UdpLink(host, port)
        self.get_logger().info(
            f'ONA link ({self.robot} #{self.robot_id}): gateways '
            + ', '.join(f'{g} ({p[0]:.1f}, {p[1]:.1f})' for g, p in self.cfg['gateways'].items())
            + f'; {len(boxes)} walls from {os.path.basename(self.world_file) or "no world file"}; '
            + (f'ONA at {host}:{port}' if transport is None else 'in-process ONA'))

        # state
        self.truth = None               # (x, y) true position (map frame = world frame at the spawn)
        self.truth_t = None
        self.odo = 0.0
        self.last_rep = None
        self.seq = 0
        self.t0 = None
        self.next_ping = None
        self.next_hb = None
        self.exec_tm = {}
        self.writer_state = 'WAIT'
        self.dropped = 0
        self.hazards = 0
        self.last_drop = None
        self.beacons = {}               # id -> {'frame': bytes, 'xy': (x, y)}
        self.deliveries = []            # [t, gw, beacon id]
        self.known = {g: set() for g in self.cfg['gateways']}
        self.rx_ok = {g: 0 for g in self.cfg['gateways']}
        self.asm = lmb2.MissionAssembler()
        self.briefings = set()
        self.downlink_frames = 0
        self.rtf = None
        self._rtf_ref = None

        brief_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.brief_pub = self.create_publisher(String, '/executor/briefing', brief_qos)
        self.status_pub = self.create_publisher(String, '/ona_link/status', 5)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, truth_topic, self.on_truth, 20)
        self.create_subscription(String, '/executor/telemetry', self.on_exec_tm, 10)
        self.create_subscription(String, '/writer/explorer_status', self.on_writer_status, 10)
        if self.live_records:
            rec_qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/writer/mission_record', self.on_mission_record, rec_qos)
        self.create_timer(0.1, self.tick)
        self._mission_loaded = False

    # ------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.truth is not None:
            d = math.hypot(x - self.truth[0], y - self.truth[1])
            if d < 1.0:
                self.odo += d
        self.truth = (x, y)
        self.truth_t = self._now()

    def on_exec_tm(self, msg):
        try:
            self.exec_tm = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def on_writer_status(self, msg):
        st = str(msg.data).split(':', 1)[0].strip().upper()
        self.writer_state = {'NAV': 'FOLLOW', 'SPIN': 'LOOK', 'INIT': 'WAIT'}.get(st, st)

    def on_mission_record(self, msg):
        try:
            d = json.loads(msg.data)
            fr = bytes.fromhex(d['frame'])
            rec = lmb2.decode_record(fr, self.key, self.net)
        except (ValueError, KeyError, TypeError, lmb2.DecodeError) as e:
            self.get_logger().warning(f'mission record not forwarded: {e}', throttle_duration_sec=5.0)
            return
        if self._add_beacon(rec['beacon_id'], fr, rec, self._now(), first_delay=(0.5, 4.0)):
            self.note_drop(rec['beacon_id'], rec['kind'])

    def _load_mission(self, now):
        self._mission_loaded = True
        if not self.mission_file:
            return
        try:
            with open(self.mission_file, encoding='utf-8') as f:
                m = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'cannot read {self.mission_file}: {e}')
            return
        a = m.get('anchor') or {}
        if a:
            self.geo = GeoFrame(a.get('lat', 36.8065), a.get('lon', 10.1815), a.get('map_yaw_deg', 0.0))
        entries = [e for e in m.get('beacons', []) if e.get('frame')]
        frames = [bytes.fromhex(e['frame']) for e in entries]
        if self.retime:
            frames = retime_frames(frames, self.key, self.net)
        n = 0
        for e, fr in zip(entries, frames):
            try:
                rec = lmb2.decode_record(fr, self.key, self.net)
            except lmb2.DecodeError:
                continue
            drop = e.get('drop')
            self._add_beacon(rec['beacon_id'], fr, rec, now, first_delay=(0.5, 10.0),
                             xy=tuple(drop[:2]) if drop else None)
            n += 1
        self.get_logger().info(f'{n} beacon records of {os.path.basename(self.mission_file)} go to the gateways')

    def _add_beacon(self, bid, frame, rec, now, first_delay, xy=None):
        if bid in self.beacons and self.beacons[bid]['frame'] == frame:
            return False
        if xy is None:
            xy = self.geo.to_xy(rec['gps']['lat'], rec['gps']['lon'])
        self.beacons[bid] = {'frame': frame, 'xy': (float(xy[0]), float(xy[1])), 'kind': rec['kind']}
        for gw in self.cfg['gateways']:
            self.deliveries.append([now + self.rng.uniform(*first_delay), gw, bid])
        return True

    # ------------------------------------------------------------ loop
    def tick(self):
        now = self._now()
        wall = time.time()
        if self.t0 is None:
            self.t0 = now
            self.next_ping = now + 1.0
            self.next_hb = now + 0.5
        if not self._mission_loaded:
            self._load_mission(now)
        self._measure_rtf(now, wall)
        # beacon records reaching the gateways (and their anti-entropy repeats)
        due = [d for d in self.deliveries if d[0] <= now]
        if due:
            self.deliveries = [d for d in self.deliveries if d[0] > now]
            relays = {b: v['xy'] for b, v in self.beacons.items()}
            for _, gw, bid in due:
                b = self.beacons.get(bid)
                if b is None:
                    continue
                self.link.send(self.radio.record_line(gw, b['frame'], b['xy'], bid, relays))
                self.known[gw].add(bid)
                self.rx_ok[gw] += 1
                self.deliveries.append([now + self.rng.uniform(40.0, 90.0), gw, bid])
        if now >= self.next_ping:
            self.next_ping = now + self.ping_period
            self._ping(now)
        if now >= self.next_hb:
            self.next_hb = now + self.hb_period
            for gw in self.cfg['gateways']:
                hb = {'gw': gw, 'ev': 'hb', 'known': len(self.known[gw]), 'rx_ok': self.rx_ok[gw], 'duty': 0.004}
                if self.advertise_speed and self.rtf is not None:
                    hb['sim_speed'] = round(self.rtf, 3)
                self.link.send(hb)
            self._status()
        for d in self.link.poll():
            if d.get('ev') == 'tx' and d.get('frame'):
                self.on_downlink(d.get('gw'), d['frame'])

    def _measure_rtf(self, now, wall):
        """Simulated seconds per real second (Gazebo in a VM often runs at 10-50 %): the ONA needs it."""
        if self._rtf_ref is None:
            self._rtf_ref = (now, wall)
            return
        dn, dw = now - self._rtf_ref[0], wall - self._rtf_ref[1]
        if dw >= 5.0:
            r = max(0.02, min(50.0, dn / dw)) if dn > 0 else None
            if r is not None:
                self.rtf = r if self.rtf is None else 0.6 * self.rtf + 0.4 * r
            self._rtf_ref = (now, wall)

    def _pose(self):
        return lookup_pose(self.tf_buffer, self.map_frame, self.base_frame, self.odom_frame)

    def _ping(self, now):
        pose = self._pose()
        truth = self.truth
        if pose is None and truth is None:
            return
        if truth is None:
            truth = (pose[0], pose[1])            # no ground truth topic: the gateways measure the SLAM pose
        self.seq = (self.seq + 1) & 0xFFFF
        mins = (now - self.t0) / 60.0
        rep = {'robot_id': self.robot_id, 'seq': self.seq, 'ts': int(time.time()),
               'odo_m': self.odo, 'battery_pct': max(5, 100 - int(mins * 2)),
               'pose_sd_m': round(0.10 + 0.012 * math.sqrt(max(self.odo, 0.0)), 2)}
        if pose is not None:
            rep.update(x=pose[0], y=pose[1], yaw=pose[2], pose_valid=True)
        else:
            rep.update(x=truth[0], y=truth[1], yaw=0.0, pose_valid=False)
        if self.robot == 'executor':
            tm = self.exec_tm
            ph = str(tm.get('phase', 'WAIT')).upper()
            rep.update(role='EXECUTOR', phase=ph if ph in EXEC_PHASES else 'GOTO',
                       mission_id=int(tm.get('mission_id') or 0), mission_ack=bool(tm.get('mission_ack')),
                       done=int(tm.get('done') or 0), total=int(tm.get('total') or 0),
                       last_beacon=tm.get('last_beacon'), stuck=str(tm.get('state', '')) == 'RECOVER')
        else:
            ph = self.writer_state if self.writer_state in WRITER_PHASES else 'FOLLOW'
            rep.update(role='WRITER', phase=ph, done=min(15, self.hazards), total=0,
                       last_beacon=self.last_drop, stuck=ph == 'RECOVER')
        frame = lmb2.encode_robot(rep, self.key, self.net)
        for line in self.radio.ping_lines(frame, truth):
            self.link.send(line)
        self.last_rep = rep

    def on_downlink(self, gw, frame_hex):
        """A frame the ONA asks a gateway to transmit (a briefing for the Executor)."""
        self.downlink_frames += 1
        if self.robot != 'executor':
            return
        pos = self.truth or (self._pose() or (0.0, 0.0))[:2]
        if gw in self.cfg['gateways'] and not self.radio.hears(pos, gw):
            return                                   # this gateway cannot reach the robot where it is
        try:
            b = briefing_from_frames(self.asm, bytes.fromhex(frame_hex), self.key, self.net, self.robot_id)
        except (ValueError, lmb2.DecodeError) as e:
            self.get_logger().warning(f'downlink frame from {gw} rejected: {e}', throttle_duration_sec=5.0)
            return
        if b is None or (b['mission_id'], b['seq']) in self.briefings:
            return
        self.briefings.add((b['mission_id'], b['seq']))
        out = String()
        out.data = json.dumps(b)
        self.brief_pub.publish(out)
        self.get_logger().info(f'briefing {b["mission_id"]} heard via {gw}: '
                               + ('ABORT' if b['abort'] else ' -> '.join(f'#{t}' for t in b['targets'])))

    def _status(self):
        m = String()
        r = self.last_rep or {}
        m.data = (f'{self.robot} #{self.robot_id} ping {self.seq} at ({r.get("x", 0):.1f}, {r.get("y", 0):.1f}) '
                  f'{r.get("phase", "?")}; records {len(self.beacons)}; lines sent {getattr(self.link, "sent", "?")}'
                  f'; briefings {len(self.briefings)}' + (f'; sim {self.rtf:.2f}x real time' if self.rtf else ''))
        self.status_pub.publish(m)

    def note_drop(self, bid, kind):
        """(Writer) a beacon was dropped - for the ROBOT frame's progress fields."""
        self.dropped += 1
        self.last_drop = bid
        if kind not in ('WAYPOINT', 'EXIT'):
            self.hazards += 1


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
    node = OnaLinkNode()
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
