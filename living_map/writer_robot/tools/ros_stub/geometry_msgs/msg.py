from std_msgs.msg import Header


class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


Point = Vector3


class Quaternion:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseWithCovariance:
    def __init__(self):
        self.pose = Pose()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class PointStamped:
    def __init__(self):
        self.header = Header()
        self.point = Point()


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()
