#!/usr/bin/env python

"""Get realsense image from realsense node."""

from __future__ import print_function
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageConverter:
    """Get realsense image and convert to cv image type."""

    def __init__(self, topic='/camera/color/image_raw', type=Image):
        """Initial cv_bridge, subscriber, and image."""
        self.__bridge = CvBridge()
        self.__img_sub = rospy.Subscriber(
            topic,
            type,
            self.__callback,
            queue_size=1)

        self.__cv_img = None
        self.img_stamp = None

    def __callback(self, data):
        try:
            self.__cv_img = self.__bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_stamp = data.header.stamp
        except CvBridgeError as e:
            print('ImageConverter_callback:', e)

        # image size, type
        (rows, cols, channels) = self.__cv_img.shape

    @property
    def cv_img(self):
        """Image getter: cv_image."""
        return self.__cv_img

    def imshow(self, event=None):
        """Show cv image."""
        if self.__cv_img is not None:
            cv2.imshow("Image", self.__cv_img)
            cv2.waitKey(10)
