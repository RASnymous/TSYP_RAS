from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance


class MapMetaData:
    def __init__(self):
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = Pose()


class OccupancyGrid:
    def __init__(self):
        self.header = Header()
        self.info = MapMetaData()
        self.data = []


class Path:
    def __init__(self):
        self.header = Header()
        self.poses = []


class Odometry:
    def __init__(self):
        self.header = Header()
        self.pose = PoseWithCovariance()
