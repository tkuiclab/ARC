#!/usr/bin/env python

"""Object detector of ros node based on darkflow."""

from __future__ import print_function
import os
import sys
import time

import rospy
import rospkg
pkg_path = rospkg.RosPack().get_path('darkflow_detect')
sys.path.append(pkg_path)
os.chdir(pkg_path)

import cv2
from image_convert import ImageConverter
from darkflow_detect.srv import Detect, DetectResponse
from darkflow.net.build import TFNet


def handle_request(req):
    """Service request callback."""
    frame = img_cvt.cv_img

    # Calculate time of detection
    start_time = time.time()
    result = tfnet.return_predict(frame)
    rospy.loginfo('Prediction time: {0:.5f}'.format(time.time() - start_time))

    res = DetectResponse([], 0.0, False)
    for info in result:
        print_info(info)

        if (info['label'] == req.object_name and
                info['confidence'] > res.confidence):
            res.confidence = info['confidence']
            res.bound_box = [
                info['topleft']['x'],
                info['topleft']['y'],
                info['bottomright']['x'],
                info['bottomright']['y']
            ]
            res.result = True
    
    print('===========================' if len(result)
        else 'Nothing was detected')
    mark_frame(frame, res.bound_box, req.object_name, res.confidence)
    return res


def print_info(info):
    """Print infomation of detecting result."""
    print('---------------------------')
    print(info['label'])
    print(info['confidence'])
    print(info['topleft'])
    print(info['bottomright'])


def mark_frame(frame, bbox, label='test', confidence=-0.1):
    """Mark the image for detecting result."""
    # If the object was detected
    if len(bbox) > 0:
        color = (100, 100, 255)
        thickness = 2
        cv2.rectangle(
            frame,
            (bbox[0], bbox[1]),
            (bbox[2], bbox[3]),
            color=color,
            thickness=thickness
        )
        cv2.putText(
            frame,
            label if confidence < 0 else label + ': {0:.3f}'.format(confidence),
            (bbox[0], bbox[1] - 10),
            0,
            .6,
            color=color,
            thickness=thickness
        )
    # Assign frame to global _img object
    global _img
    _img.frame = frame


def show_detection(event):
    """Show result of image for timer using."""
    if _img.refresh:
        cv2.imshow('Prediction', _img.frame)
    cv2.waitKey(10)


def prepare_network():
    """Use prediction of net at program start because first time is slow."""
    import numpy as np
    tfnet.return_predict(np.zeros((2, 2, 3)))


# Using @foo.setter need to inherit class of "object"
class Image(object):
    """For checking the frame is newest."""
    _frame = None
    _refresh = False

    @property
    def frame(self):
        """Frame getter: cv_image."""
        self._refresh = False
        return self._frame

    @frame.setter
    def frame(self, img):
        """Frame setter: cv_image."""
        self._frame = img
        self._refresh = True

    @property
    def refresh(self):
        """Refresh getter."""
        return self._refresh

_img = Image()

# Options for net building
options = {
    "model": "cfg/yolo-new.cfg",   # model of net
    "backup": "ckpt/",              # directory of ckpt (training result)
    "load": -1,                     # which ckpt will be loaded. -1 represent the last ckpt
    "threshold": -0.1,              # threshold for confidence
    "gpu": 0.5                       # gpu using rate
}
tfnet = TFNet(options)
prepare_network()


if __name__ == '__main__':
    rospy.init_node('object_detect', anonymous=True)
    img_topic = (
        '/camera/rgb/image_raw' if len(sys.argv) < 2
        else sys.argv[1]
    )
    img_cvt = ImageConverter(img_topic)

    # Show result of detection for every 100ms if the frame is fresh
    rospy.Timer(rospy.Duration(.1), show_detection)
    rospy.Service('detect', Detect, handle_request)
    rospy.loginfo("Topic of image is '{}'.".format(img_topic))
    rospy.loginfo('Object detector is running.')
    rospy.spin()
