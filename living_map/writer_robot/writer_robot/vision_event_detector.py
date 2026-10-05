#!/usr/bin/env python3
"""
Vision Event Detector - Writer Robot (TSYP14 Living Map)

Real computer-vision event detection from the RGB-D camera. Three event
types are recognised by colour signature (a stand-in for trained hazard
classes; swap the HSV masks for a YOLO/MobileNet head on Jetson later):

    - "radiation"  : magenta / purple radiation-source block
    - "fire"       : red fire / heat-hazard block
    - "gas"        : yellow gas-leak block

For every frame where a hazard blob is big enough (min_area_px) and has a
valid depth:
  - range and bearing come from the pinhole model of the camera and the
    median depth over the blob (depth = distance along the optical axis)
  - the hazard is projected into the SLAM `map` frame with the robot pose
    (TF map -> base_footprint at the image time)
  - /writer/hazard  (String JSON, every sighting, max 5 Hz per type)
        {"type", "x", "y", "range", "bearing", "area", "stamp"}
    -> the explorer turns it into a keep-out zone and turns back
  - /writer/events  (String JSON, ONCE per hazard, within drop_range_m)
    -> the beacon node drops a beacon for it (event_x/event_y = hazard)
  - /writer/vision_target (PointStamped, base_link) for visualisation

The Writer MARKS hazards from a safe distance - it never drives into them.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
import tf2_ros

from writer_robot.tf_pose import lookup_pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    _HAVE_CV = True
except ImportError:
    _HAVE_CV = False


# HSV colour gates (OpenCV H in [0,179]). Tune to your world markers.
# Rubble (brown, low saturation), walls (grey) and the dropped beacons
# (green body, blue LED) fall outside every gate.
COLOR_GATES = {
    'radiation': [((140, 90, 80), (170, 255, 255))],                                    # magenta
    'fire':      [((0, 120, 90), (10, 255, 255)), ((170, 120, 90), (179, 255, 255))],  # red (wraps)
    'gas':       [((18, 150, 90), (35, 255, 255))],                                     # yellow
}


class VisionEventDetector(Node):
    def __init__(self):
        super().__init__('vision_event_detector')

        num = ParameterDescriptor(dynamic_typing=True)    # 800 or 800.0 both accepted
        self.declare_parameter('min_area_px', 800, num)   # ~3.7 m for a 0.3 x 0.6 m block
        self.declare_parameter('drop_range_m', 4.0, num)  # record (beacon) hazards seen within this
        self.declare_parameter('cooldown_s', 3.0, num)
        self.declare_parameter('hfov_rad', 1.089, num)
        self.declare_parameter('camera_offset_m', 0.213, num)  # camera ahead of base_footprint
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('dedup_radius_m', 1.5, num)     # one beacon per hazard within this
        self.declare_parameter('hazard_rate_hz', 5.0, num)
        self.declare_parameter('camera_height_m', 0.135, num)  # optical centre above the floor
        self.declare_parameter('hazard_width_m', 0.30, num)    # size of the hazard blocks
        self.min_area = float(self.get_parameter('min_area_px').value)
        self.drop_range = float(self.get_parameter('drop_range_m').value)
        self.cooldown = float(self.get_parameter('cooldown_s').value)
        self.hfov = float(self.get_parameter('hfov_rad').value)
        self.cam_off = float(self.get_parameter('camera_offset_m').value)
        self.cam_h = float(self.get_parameter('camera_height_m').value)
        self.haz_w = float(self.get_parameter('hazard_width_m').value)
        # health counters, logged every 10 s (see /tmp/vision.log)
        self.n_frames = 0
        self.n_depth = 0
        self.n_seen = {}
        self.n_fallback = 0
        self.last_report = None
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.dedup_radius = float(self.get_parameter('dedup_radius_m').value)
        self.hazard_period = 1.0 / float(self.get_parameter('hazard_rate_hz').value)

        if not _HAVE_CV:
            self.get_logger().error(
                'OpenCV / cv_bridge not available - install ros-<distro>-cv-bridge '
                'and python3-opencv. Node will idle.')
            return

        self.bridge = CvBridge()
        self.odom = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.depth = None
        self.last_trigger = {k: -1e9 for k in COLOR_GATES}
        self.last_hazard = {k: -1e9 for k in COLOR_GATES}
        self.marked = []      # (event_x, event_y, type) already beaconed - drop once per hazard

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 5)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, 5)

        self.event_pub = self.create_publisher(String, '/writer/events', 10)
        self.hazard_pub = self.create_publisher(String, '/writer/hazard', 20)
        self.target_pub = self.create_publisher(PointStamped, '/writer/vision_target', 10)

        self.create_timer(10.0, self._report)
        self.get_logger().info('Vision Event Detector started (radiation + fire + gas).')

    # ---------------- pose (map frame) ----------------
    def _map_pose(self, stamp=None):
        """Robot pose in the map frame (x, y, theta) at the image time if
        possible, else the latest; falls back to /odom."""
        p = lookup_pose(self.tf_buffer, self.map_frame, self.base_frame, 'odom', stamp)
        if p is not None:
            return p
        return (self.odom['x'], self.odom['y'], self.odom['theta'])

    # ---------------- callbacks ----------------
    def on_odom(self, msg: Odometry):
        self.odom['x'] = msg.pose.pose.position.x
        self.odom['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odom['theta'] = math.atan2(siny, cosy)

    def on_depth(self, msg: Image):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.n_depth += 1
        except Exception:  # noqa: BLE001
            self.depth = None

    def _report(self):
        seen = ', '.join(f'{k} {v}' for k, v in sorted(self.n_seen.items())) or 'nothing'
        self.get_logger().info(
            f'[health] last 10 s: {self.n_frames} colour frames, {self.n_depth} depth frames, '
            f'hazard frames: {seen}' + (f' ({self.n_fallback} ranged without depth)' if self.n_fallback else ''))
        if self.n_frames == 0:
            self.get_logger().warning('[health] NO camera images on /camera/image_raw - is the bridge running?')
        elif self.n_depth == 0:
            self.get_logger().warning('[health] no depth images on /camera/depth/image_raw - '
                                      'using the floor-contact range estimate instead')
        self.n_frames = self.n_depth = self.n_fallback = 0
        self.n_seen = {}

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'cv_bridge convert failed: {e}')
            return

        self.n_frames += 1
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, w = bgr.shape[:2]
        fx = (w / 2.0) / math.tan(self.hfov / 2.0)      # focal length in pixels
        now = self.get_clock().now().nanoseconds * 1e-9   # sim time with use_sim_time
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        # use the depth image only if it belongs to THIS colour image (Gazebo
        # stamps both alike): an older one would put the blob on the wall
        # behind the block while the robot turns
        img_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_ok = self.depth is not None and abs(img_t - getattr(self, 'depth_t', img_t)) <= 0.12

        for etype, ranges in COLOR_GATES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']

            # pinhole model: bearing of the blob centre (left = positive)
            bearing = math.atan2((w / 2.0) - cx, fx)
            depth = self._blob_depth(c, h, w)
            if depth is None:
                # no usable depth: range from where the block stands on the floor
                depth = self._ground_depth(c, h, w, fx)
                if depth is None:
                    continue
                self.n_fallback += 1
            self.n_seen[etype] = self.n_seen.get(etype, 0) + 1
            rng = depth / max(math.cos(bearing), 0.2)   # along the ray

            hx, hy, rx, ry, rth = self._hazard_map_xy(depth, bearing, stamp)
            self._publish_target(bearing, rng)
            if now - self.last_hazard[etype] >= self.hazard_period:
                self.last_hazard[etype] = now
                self._publish_hazard(etype, hx, hy, rng, bearing, area)
            # RECORD (drop a beacon) once per hazard, when close enough
            if rng <= self.drop_range and (now - self.last_trigger[etype]) >= self.cooldown:
                if self._publish_event(etype, area, rng, bearing, hx, hy, rx, ry, rth):
                    self.last_trigger[etype] = now

    # ---------------- helpers ----------------
    def _ground_depth(self, contour, h, w, fx):
        """Depth (m, along the optical axis) without a depth image: the blob's
        lowest row is where the block meets the floor, so with the camera
        `camera_height_m` above a flat floor, depth = height * f / (row - cy).
        If the bottom is cut off by the image edge, use the block's width."""
        x, y, bw, bh = cv2.boundingRect(contour)
        bottom = y + bh
        cy = h / 2.0
        if bottom < h - 2 and bottom - cy > 3:
            return self.cam_h * fx / (bottom - cy)
        if bw >= 8:
            return self.haz_w * fx / bw
        return None

    def _blob_depth(self, contour, h, w):
        """Median depth (m, along the optical axis) over the blob, or None."""
        if self.depth is None or not getattr(self, 'depth_ok', True):
            return None
        d = self.depth
        if d.shape[:2] != (h, w):
            return None
        blob = np.zeros((h, w), np.uint8)
        cv2.drawContours(blob, [contour], -1, 255, -1)
        vals = d[blob == 255]
        if vals.dtype == np.uint16:       # 16UC1 depth images are in millimetres
            vals = vals.astype(np.float32) / 1000.0
        vals = vals[np.isfinite(vals) & (vals > 0.05)]
        if vals.size < 20:
            return None
        return float(np.median(vals))

    def _hazard_map_xy(self, depth, bearing, stamp):
        """Project the hazard into the map frame from the robot's map pose."""
        rx, ry, rth = self._map_pose(stamp)
        # camera frame -> base: forward = depth, left = depth * tan(bearing)
        bx = self.cam_off + depth
        by = depth * math.tan(bearing)
        hx = rx + bx * math.cos(rth) - by * math.sin(rth)
        hy = ry + bx * math.sin(rth) + by * math.cos(rth)
        return hx, hy, rx, ry, rth

    def _publish_target(self, bearing, rng):
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = self.cam_off + rng * math.cos(bearing)
        pt.point.y = rng * math.sin(bearing)
        pt.point.z = 0.0
        self.target_pub.publish(pt)

    def _publish_hazard(self, etype, hx, hy, rng, bearing, area):
        msg = String()
        msg.data = json.dumps({
            'type': etype, 'x': round(hx, 3), 'y': round(hy, 3),
            'range': round(rng, 2), 'bearing': round(bearing, 3),
            'area': int(area), 'stamp': self.get_clock().now().nanoseconds * 1e-9,
        })
        self.hazard_pub.publish(msg)

    def _publish_event(self, etype, area, rng, bearing, hx, hy, rx, ry, rth):
        # ONE beacon per hazard: skip if this event was already marked nearby
        for mx, my, mt in self.marked:
            if mt == etype and math.hypot(hx - mx, hy - my) < self.dedup_radius:
                return False
        self.marked.append((hx, hy, etype))
        value = round(min(1.0, area / 40000.0), 3)
        payload = {
            'event_type': etype,
            'value': value,
            'range_m': round(rng, 2),
            'bearing_rad': round(bearing, 3),
            # x,y = the ROBOT's position when it saw the event = where the
            # physical beacon is dropped. event_x/event_y = the hazard itself.
            'x': round(rx, 3), 'y': round(ry, 3), 'theta': round(rth, 3),
            'event_x': round(hx, 3), 'event_y': round(hy, 3),
            'timestamp': time.time(),
            'source': 'vision',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)
        self.get_logger().warning(f'VISION EVENT -> {payload}')
        return True


try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    class ExternalShutdownException(Exception):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionEventDetector()
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
