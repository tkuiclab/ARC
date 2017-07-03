#!/usr/bin/env python

"""Object detector of ros node based on darkflow."""

from __future__ import print_function
import os
import sys
import copy
import time

import rospy
import rospkg
pkg_path = rospkg.RosPack().get_path('darkflow_detect')
sys.path.append(pkg_path)
os.chdir(pkg_path)

import cv2
from image_convert import ImageConverter
from image_convert import save_img, get_now
from darkflow_detect.srv import Detect, DetectResponse
from darkflow_detect.msg import Detected
from darkflow.net.build import TFNet
from convert_label.convert import offical2Our, our2Offical, colors
from get_obj_info import info_dict


def handle_request(req):
    """Service request callback."""
    frame = img_cvt.cv_img
    # Checking image is coming
    if frame is None:
        rospy.logwarn("There is no image! Does realsense launch?")
        return DetectResponse([], False)

    cvted_name = offical2Our(req.object_name)
    # Checking converted name
    if not cvted_name:
        global _img
        _img.predi = frame
        return DetectResponse([], False)

    # Calculate time of detection
    start_time = time.time()
    result = tfnet.return_predict(frame)
    rospy.loginfo('Prediction time: {0:.5f}'.format(time.time() - start_time))

    detectedList = list()
    for info in result:
        print_info(info)
        # Checking detected object
        if cvted_name == 'all':
            detectedList.append(detectedInfoToMsg(info))
        elif cvted_name == info['label']:
            if len(detectedList) > 0:
                if info['confidence'] > detectedList[0].confidence:
                    detectedList[0] = detectedInfoToMsg(info)
            else:
                detectedList.append(detectedInfoToMsg(info))

    res = DetectResponse([], False)
    if len(detectedList):
        res.detected = detectedList
        res.result = True

    mark_frame(frame, detectedList)
    print('===========================' if len(result)
          else 'Nothing was detected')

    return res


def detectedInfoToMsg(info):
    """Convert detected infomations to message type."""
    msg = Detected()
    msg.object_name = our2Offical(info['label'])
    msg.type = info_dict[msg.object_name].type
    msg.confidence = info['confidence']
    msg.bound_box = [
        info['topleft']['x'],
        info['topleft']['y'],
        info['bottomright']['x'],
        info['bottomright']['y']
    ]
    return msg


def draw_bbox(frame, bbox, label='', confidence=-0.1):
    """Drawing bbox on image."""
    # If the object was detected
    if len(bbox):
        color = colors[label] if label != '' else (100, 100, 255)
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
            label if confidence < 0 else label +
            ': {0:.3f}'.format(confidence),
            (bbox[0], bbox[1] - 10),
            0,
            .55,
            color=color,
            thickness=thickness
        )


def mark_frame(frame, detected):
    """Mark the image for detecting result."""
    # Assign original frame to global _img object
    global _img
    _img.frame = copy.deepcopy(frame)
    # Drawing all of bbox
    for result in detected:
        draw_bbox(frame, result.bound_box,
                  result.object_name, result.confidence)
    # Assign frame of prediction to global _img object
    _img.predi = frame


def print_info(info):
    """Print infomation of detecting result."""
    label_name = our2Offical(info['label'])
    print('---------------------------')
    print(label_name)
    print(info_dict[label_name].type)
    print(info['confidence'])
    print(info['topleft'])
    print(info['bottomright'])


def show_detection(event):
    """Show result of image for timer using."""
    if _img.refresh:
        cv2.imshow('Prediction', _img.predi)
    # Pressing <space> key
    if cv2.waitKey(10) == 32:
        save_img(_img.frame)


def prepare_network():
    """Use prediction of net at program start because first time is slow."""
    import numpy as np
    tfnet.return_predict(np.zeros((2, 2, 3)))


# Using @foo.setter need to inherit class of "object"
class Image(object):
    """For checking the frame is newest."""
    _frame = None
    _predi = None
    _refresh = False

    @property
    def frame(self):
        """Original frame getter: cv_image."""
        self._refresh = False
        return self._frame

    @frame.setter
    def frame(self, img):
        """Original frame setter: cv_image."""
        self._frame = img
        self._refresh = True

    @property
    def refresh(self):
        """Refresh getter."""
        return self._refresh

    @property
    def predi(self):
        """Frame of predication getter: cv_image."""
        self._refresh = False
        return self._predi

    @predi.setter
    def predi(self, img):
        """Frame of predication setter: cv_image."""
        self._predi = img
        self._refresh = True


_img = Image()

# Options for net building
options = {
    "model": "cfg/yolo-new.cfg",    # model of net
    "backup": "ckpt/",              # directory of ckpt (training result)
    "load": -1,                 # which ckpt will be loaded. -1 represent the last ckpt
    "threshold": 0.0,               # threshold for confidence
    "gpu": 1.0                      # gpu using rate
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
