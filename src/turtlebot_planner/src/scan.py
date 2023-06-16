#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class VisualScanNode:
    def __init__(self):
        rospy.init_node('scan')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback)
        self.result_pub = rospy.Publisher('/witsdetector', String, queue_size=10)
        self.height = 100
        self.width = 100
        self.global_frame = np.zeros((self.height,self.width,3), np.uint8)

    def image_callback(self, data):
        try:
            # Convert to opencv image
            self.global_frame = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Error: {}".format(e))
            return

        # Convert image to HSV and apply color thresholding
        hsv_image = cv2.cvtColor(self.global_frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([60, 100, 100], np.uint8)
        upper_green = np.array([60, 255, 255], np.uint8)
        mask = cv2.inRange(hsv_image, lower_green, upper_green) # green vals > 0 else 0

        # Check for green pixels in the image using mask
        if np.any(mask > 0):
            # Green object detected
            self.result_pub.publish('Yes')
        else:
            # No green object detected
            self.result_pub.publish('No')

if __name__ == '__main__':
    scan_node = VisualScanNode()
    rospy.spin()
