class Duration:
    def __init__(self, seconds=0.0, nanoseconds=None):
        self.nanoseconds = int(nanoseconds if nanoseconds is not None else seconds * 1e9)
