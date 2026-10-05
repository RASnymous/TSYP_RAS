import enum


class ReliabilityPolicy(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class HistoryPolicy(enum.Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:
    def __init__(self, **kw):
        self.__dict__.update(kw)


qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
