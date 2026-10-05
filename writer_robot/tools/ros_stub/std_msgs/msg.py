from builtin_interfaces.msg import Time


class Header:
    def __init__(self):
        self.stamp = Time()
        self.frame_id = ''


class String:
    def __init__(self, data=''):
        self.data = data


class Float64:
    def __init__(self, data=0.0):
        self.data = data


class ColorRGBA:
    def __init__(self):
        self.r = self.g = self.b = self.a = 0.0
