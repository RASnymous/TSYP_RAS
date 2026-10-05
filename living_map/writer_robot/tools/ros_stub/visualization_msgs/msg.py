from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3


class Marker:
    ARROW, CUBE, SPHERE, CYLINDER = 0, 1, 2, 3
    LINE_STRIP, LINE_LIST = 4, 5
    TEXT_VIEW_FACING = 9
    ADD, DELETE, DELETEALL = 0, 2, 3

    def __init__(self):
        self.header = Header()
        self.ns = ''
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = Pose()
        self.scale = Vector3()
        self.color = ColorRGBA()
        self.text = ''
        self.points = []


class MarkerArray:
    def __init__(self):
        self.markers = []
