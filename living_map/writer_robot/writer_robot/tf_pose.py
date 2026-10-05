"""Robot pose in the map frame from TF, as fresh as possible.

slam_toolbox publishes map -> odom only as often as it processes scans (and
more slowly when Gazebo runs below real time), while odom -> base_footprint
comes at 30 Hz (Gazebo's odometry, or the EKF with odom_source:=ekf). A single lookup map -> base_footprint at
"latest" returns the last time BOTH are known, i.e. a pose that can lag behind
the moving robot. Composing the latest map -> odom with the latest
odom -> base_footprint gives the current pose with the latest SLAM correction.
"""
import math

import rclpy
import tf2_ros


def _xyyaw(t):
    q = t.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def _compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    yaw = math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw))
    return ax + c * bx - s * by, ay + s * bx + c * by, yaw


def lookup_pose(buf, map_frame, base_frame, odom_frame='odom', stamp=None):
    """(x, y, yaw) of base_frame in map_frame, or None.
    stamp: pose at that time if TF has it (e.g. when a scan was taken)."""
    latest = rclpy.time.Time()
    if stamp is not None:
        try:
            return _xyyaw(buf.lookup_transform(map_frame, base_frame, stamp))
        except tf2_ros.TransformException:
            pass
        # map -> odom not known at that time: the latest one (it changes
        # slowly) with odom -> base_footprint AT that time (30 Hz, it follows
        # the robot's fast turns)
        try:
            return _compose(_xyyaw(buf.lookup_transform(map_frame, odom_frame, latest)),
                            _xyyaw(buf.lookup_transform(odom_frame, base_frame, stamp)))
        except tf2_ros.TransformException:
            pass
    try:
        return _compose(_xyyaw(buf.lookup_transform(map_frame, odom_frame, latest)),
                        _xyyaw(buf.lookup_transform(odom_frame, base_frame, latest)))
    except tf2_ros.TransformException:
        pass
    try:
        return _xyyaw(buf.lookup_transform(map_frame, base_frame, latest))
    except tf2_ros.TransformException:
        return None
