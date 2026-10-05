"""tf2_ros stand-in: the harness sets POSE[(target, source)] = (x, y, yaw)."""
import math

POSE = {}


class TransformException(Exception):
    pass


class LookupException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class _V:
    pass


class Buffer:
    def lookup_transform(self, target, source, time, timeout=None):
        p = POSE.get((target, source))
        if p is None:
            raise LookupException(f'{target} -> {source} not available')
        x, y, yaw = p
        t = _V()
        t.transform = _V()
        t.transform.translation = _V()
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, 0.0
        t.transform.rotation = _V()
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        return t


class TransformListener:
    def __init__(self, buffer, node):
        pass
