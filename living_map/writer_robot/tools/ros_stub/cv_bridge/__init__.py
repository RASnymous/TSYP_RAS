class CvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        return msg._array
