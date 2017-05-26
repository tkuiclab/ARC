#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import cv2

import rospy
import rospkg
pkg_path = rospkg.RosPack().get_path('darkflow_detect')
sys.path.append(pkg_path)
os.chdir(pkg_path)

from image_convert import ImageConverter
from darkflow_detect.srv import Detect, DetectResponse
from darkflow.net.build import TFNet


def handle_request(req):
    frame = img_cvt.cv_img
    result = tfnet.return_predict(frame)

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
    print()
    if res.result is True:
        draw_bbox(frame, res.bound_box, info['label'], res.confidence)
    return res


def print_info(info):
    print('---------------------------')
    print(info['label'])
    print(info['confidence'])
    print(info['topleft'])
    print(info['bottomright'])


def draw_bbox(frame, bbox, label='test', confidence=-0.1):
    import copy
    global _img
    _img = copy.deepcopy(frame)
    color = (100, 100, 255)
    thickness = 2
    cv2.rectangle(
        _img,
        (bbox[0], bbox[1]),
        (bbox[2], bbox[3]),
        color=color,
        thickness=thickness
    )

    text_height = float(bbox[3] - bbox[1]) / _img.shape[0] * 1.5
    cv2.putText(
        _img,
        label if confidence < 0 else label + ": {0:.2f}".format(confidence),
        (bbox[0], bbox[1] - 10),
        0,
        text_height if text_height > 0.5 else 0.5,
        color=color,
        thickness=thickness
    )


def show_detection(event):
    if _img is None:
        return
    cv2.imshow('Prediction', _img)
    cv2.waitKey(10)


options = {
    "model": "cfg/yolo-test.cfg",
    "backup": "ckpt/",
    "load": -1,
    "threshold": -0.1,
    "gpu": .8
}
tfnet = TFNet(options)

rospy.init_node('object_detect', anonymous=True)
img_cvt = ImageConverter()

_img = None
rospy.Timer(rospy.Duration(.5), show_detection)
rospy.Service('detect', Detect, handle_request)
rospy.loginfo('Object detector is running.')
rospy.spin()
