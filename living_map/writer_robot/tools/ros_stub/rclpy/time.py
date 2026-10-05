class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class Time:
    def __init__(self, seconds=0.0, nanoseconds=None):
        self.nanoseconds = int(nanoseconds if nanoseconds is not None else seconds * 1e9)

    def to_msg(self):
        return _Stamp(self.nanoseconds // 1_000_000_000, self.nanoseconds % 1_000_000_000)

    @staticmethod
    def from_msg(m):
        return Time(nanoseconds=m.sec * 1_000_000_000 + m.nanosec)

    def __sub__(self, other):
        from .duration import Duration
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)
