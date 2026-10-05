from std_msgs.msg import Header


class LaserScan:
    def __init__(self):
        self.header = Header()
        self.angle_min = self.angle_max = self.angle_increment = 0.0
        self.range_min = self.range_max = 0.0
        self.ranges = []


class Image:
    """The stub carries the numpy array directly in `_array`."""
    def __init__(self, array=None, encoding='bgr8'):
        self.header = Header()
        self.encoding = encoding
        self._array = array
