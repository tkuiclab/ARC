#!/usr/bin/env python

"""Get realsense image from realsense node."""

from __future__ import print_function
import os
import sys
import time

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageConverter(object):
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
        """Callback for come in data of image raw."""
        try:
            self.__cv_img = self.__bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_stamp = data.header.stamp
        except CvBridgeError as e:
            print('ImageConverter_callback:', e)
        # image size, type
        #(rows, cols, channels) = self.__cv_img.shape

    @property
    def cv_img(self):
        """Image getter: cv_image."""
        return self.__cv_img

    def imshow(self, event=None):
        """Show cv image."""
        if self.__cv_img is not None:
            cv2.imshow("Image", self.__cv_img)
            # Pressing <space> key
            if cv2.waitKey(10) == 32:
                save_img(self.__cv_img)


def save_img(img):
    """Saving image at home, folder name: date, file name: time."""
    _path = os.path.expanduser(os.path.join('~', get_now('d')))
    if not os.path.exists(_path):
        os.makedirs(_path)
    cv2.imwrite(os.path.join(_path, '{}.jpg'.format(get_now('t'))), img)


def get_now(arg):
    """Getting now of date or time."""
    import datetime
    now = datetime.datetime.now()
    if arg == 'd':
        return (
            '{:04d}'.format(now.year)+
            '{:02d}'.format(now.month)+
            '{:02d}'.format(now.day)
        )
    elif arg == 't':
        return (
            '{:02d}'.format(now.hour)+
            '{:02d}'.format(now.minute)+
            '{:02d}'.format(now.second)+
            '{:02d}'.format(now.microsecond)
        )


if __name__ == '__main__':
    rospy.init_node('show_image', anonymous=True)
    img_topic = (
        '/camera/rgb/image_raw' if len(sys.argv) < 2
        else sys.argv[1]
    )
    img_cvt = ImageConverter(img_topic)

    # Show result of detection for every 100ms if the frame is fresh
    rospy.Timer(rospy.Duration(.1), img_cvt.imshow)
    rospy.loginfo("Topic of image is '{}'.".format(img_topic))
    rospy.spin()
