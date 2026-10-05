#!/usr/bin/env python3
"""
beacon_record_node.py - Writer robot: turn every dropped beacon into an LMB2 record
====================================================================================
Subscribes to /writer/beacon_dropped (JSON String from beacon_drop_node) and, for
every beacon the Writer deposits, builds the compact record that beacon will
carry on the air (44-byte frame, HMAC-sealed):

    id        the beacon number
    kind      VICTIM / GAS / RADIATION / THERMAL (fire) / ... / WAYPOINT (trail)
    position  where the EVENT is (the hazard's projected map position from the
              camera), or where the beacon lies for trail beacons; map x/y ->
              lat/lon around an anchor; err_m from a SLAM drift model
    severity  vision strength 0..1 -> 0..100
    next      the way out: the beacon this one links to ("link_id" from
              beacon_drop_node v7: the last beacon the Writer dropped or drove
              past; older drops: the previously dropped beacon), with distance
              and compass bearing measured from this beacon's drop point
    ts, ttl, half-life   now, and the per-kind aging defaults (gas fades in
              minutes, an obstruction stays valid for days)

Outputs (any combination):
    /writer/beacon_record      std_msgs/String {"record": {...}, "frame": "<hex>"}
    dashboard:=http://IP:3000  POSTs each record to the Command Post (as if a
                               perfect radio network delivered it) + heartbeat
    serial:=/dev/ttyUSB0       provisions a REAL beacon over USB before release:
                               TIME, PROV <record>, SEED <route so far>

Run (sourced ROS 2 terminal, while the Writer runs):
    python3 beacon_record_node.py --ros-args -p dashboard:=http://10.0.2.2:3000
Parameters: anchor_lat, anchor_lon, map_yaw_deg, key ('demo' or 32 hex), net_id,
            err0_m, err_per_m, vision_confidence, dashboard, gateway_id, serial.
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'python'))
from beaconnet import proto  # noqa: E402
from beaconnet.dashboard import Poster, payload_from_record  # noqa: E402

M_PER_DEG_LAT = 111320.0
KIND_OF_EVENT = {'waypoint': 'WAYPOINT', 'trail': 'WAYPOINT', 'fire': 'THERMAL', 'thermal': 'THERMAL',
                 'radiation': 'RADIATION', 'victim': 'VICTIM', 'gas': 'GAS', 'obstruction': 'OBSTRUCTION',
                 'rubble': 'OBSTRUCTION', 'structural': 'STRUCTURAL', 'exit': 'EXIT'}


class BeaconRecordNode(Node):
    def __init__(self):
        super().__init__('beacon_record_node')
        p = self.declare_parameter
        p('anchor_lat', 36.8065)
        p('anchor_lon', 10.1815)
        p('map_yaw_deg', 0.0)          # compass direction of the map +x axis, 0 = east
        p('key', 'demo')
        p('net_id', 0x2A)
        p('err0_m', 0.3)               # position error at the start
        p('err_per_m', 0.02)           # SLAM drift: +2 cm per metre driven
        p('vision_confidence', 0.9)
        p('dashboard', '')
        p('gateway_id', 'writer')
        p('serial', '')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.lat0, self.lon0 = float(g('anchor_lat')), float(g('anchor_lon'))
        self.yaw = math.radians(float(g('map_yaw_deg')))
        k = str(g('key'))
        self.key = proto.DEMO_KEY if k == 'demo' else proto.key_from_hex(k)
        self.net = int(g('net_id'))
        self.err0, self.err_k = float(g('err0_m')), float(g('err_per_m'))
        self.vconf = float(g('vision_confidence'))
        self.gw = str(g('gateway_id'))

        self.path_m = 0.0              # distance driven (odometry)
        self.have_odom = False
        self.last_xy = None
        self.prev = None               # (id, drop_x, drop_y) of the last beacon
        self.drops = {}                # id -> (drop_x, drop_y)
        self.records = []              # every record provisioned so far (the route)

        self.pub = self.create_publisher(String, '/writer/beacon_record', 10)
        self.create_subscription(String, '/writer/beacon_dropped', self.on_drop, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

        self.poster = None
        if g('dashboard'):
            self.poster = Poster(str(g('dashboard')), log=lambda m: self.get_logger().warning(m))
            self.create_timer(4.0, self.heartbeat)
        self.ser = None
        self.ser_lock = threading.Lock()
        if g('serial'):
            import serial  # needs: pip install pyserial
            self.ser = serial.Serial(str(g('serial')), 115200, timeout=0.5)
            time.sleep(2.0)
        self.get_logger().info(
            f'LMB2 record builder: anchor {self.lat0},{self.lon0}, net 0x{self.net:02X}, '
            f"key {'DEMO' if self.key == proto.DEMO_KEY else 'mission'}"
            + (f', dashboard {g("dashboard")}' if self.poster else '') + (f', serial {g("serial")}' if self.ser else ''))

    # ------------------------------------------------------------------ geometry
    def to_en(self, x, y):
        """map x/y -> east/north metres"""
        return (x * math.cos(self.yaw) - y * math.sin(self.yaw),
                x * math.sin(self.yaw) + y * math.cos(self.yaw))

    def to_latlon(self, x, y):
        e, n = self.to_en(x, y)
        return (self.lat0 + n / M_PER_DEG_LAT,
                self.lon0 + e / (M_PER_DEG_LAT * math.cos(math.radians(self.lat0))))

    def on_odom(self, msg):
        self.have_odom = True
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < 1.0:                # ignore teleports / resets
                self.path_m += d
        self.last_xy = (x, y)

    # ------------------------------------------------------------------ records
    def on_drop(self, msg):
        try:
            b = json.loads(msg.data)
            bid = int(b['beacon_id'])
            dx, dy = float(b['x']), float(b['y'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('beacon_dropped without id/x/y - skipped')
            return
        et = str(b.get('event_type', 'waypoint')).lower()
        kind = KIND_OF_EVENT.get(et, 'WAYPOINT')
        hazard = kind not in ('WAYPOINT', 'EXIT')
        ex, ey = dx, dy
        projected = b.get('event_x') is not None and b.get('event_y') is not None
        if projected:
            ex, ey = float(b['event_x']), float(b['event_y'])
        lat, lon = self.to_latlon(ex, ey)
        path = self.path_m if self.have_odom else sum(r.get('_leg', 0.0) for r in self.records)
        if not self.have_odom and self.prev is not None:        # no odometry: use the drop chain
            path += math.hypot(self.prev[1] - dx, self.prev[2] - dy)
        err = self.err0 + self.err_k * path
        if projected:
            err += 0.25 * float(b.get('range_m') or math.hypot(ex - dx, ey - dy))   # camera range error
        rec = {
            'beacon_id': bid,
            'kind': kind,
            'gps': {'lat': round(lat, 7), 'lon': round(lon, 7), 'err_m': round(min(err, 25.4), 1), 'src': 'slam'},
            'severity': int(round(100 * max(0.0, min(1.0, float(b.get('value') or 0.0))))) if hazard else 0,
            'confidence': self.vconf if hazard else 1.0,
            'ts': int(time.time()),
        }
        leg = 0.0
        link = b.get('link_id', 'none-given')
        parent = None
        if link == 'none-given':
            parent = self.prev                                   # older Writer: chain of drops
        elif link is not None and int(link) in self.drops:
            parent = (int(link),) + self.drops[int(link)]        # v7: explicit way-out link
        if parent is not None:
            pid, px, py = parent
            e, n = self.to_en(px - dx, py - dy)
            leg = math.hypot(e, n)
            rec['next'] = {'id': pid, 'dist_m': round(leg, 1),
                           'bearing_deg': round(math.degrees(math.atan2(e, n)) % 360.0, 1)}
        try:
            r = proto.record_from_json(rec)
            frame = proto.encode(proto.Frame(rec=r, net_id=self.net, relay_id=bid), self.key)
        except proto.ProtoError as ex_:
            self.get_logger().error(f'record #{bid} invalid ({ex_}) - skipped')
            return
        self.prev = (bid, dx, dy)
        self.drops[bid] = (dx, dy)
        self.records.append({**rec, '_leg': leg})

        out = String()
        out.data = json.dumps({'record': rec, 'frame': frame.hex(), 'bytes': len(frame)})
        self.pub.publish(out)
        nxt = rec.get('next')
        self.get_logger().info(
            f"Beacon #{bid} {kind:<9} {len(frame)} B  {frame.hex()[:24]}...  "
            f"err {rec['gps']['err_m']} m" + (f"  next #{nxt['id']} {nxt['dist_m']} m @ {nxt['bearing_deg']:.0f} deg" if nxt else ''))

        if self.poster:
            self.poster.post('/api/beacon', payload_from_record(r, self.gw, ev='new', hops=0, relay=bid))
        if self.ser:
            threading.Thread(target=self.provision, args=(rec,), daemon=True).start()

    def provision(self, rec):
        """real beacon on USB: clock, own record, then the route so far (SEED)"""
        with self.ser_lock:
            w = lambda line: self.ser.write((line + '\n').encode())  # noqa: E731
            w(f'TIME {int(time.time())}')
            w('PROV ' + json.dumps(rec, separators=(',', ':')))
            time.sleep(0.3)
            for old in self.records[-48:-1]:
                w('SEED ' + json.dumps({k: v for k, v in old.items() if not k.startswith('_')}, separators=(',', ':')))
                time.sleep(0.05)
            reply = self.ser.read(4096).decode('utf-8', 'replace')
            ok = 'OK PROV' in reply
            (self.get_logger().info if ok else self.get_logger().error)(
                f"serial provisioning #{rec['beacon_id']}: {'OK' if ok else 'NO ACK: ' + reply[:120]}")

    def heartbeat(self):
        self.poster.post('/api/network-health', {'gateway_id': self.gw, 'status': 'online',
                                                 'timestamp': time.time(), 'known': len(self.records)})


def main():
    rclpy.init()
    node = BeaconRecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
