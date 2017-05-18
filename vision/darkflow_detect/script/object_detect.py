#!/usr/bin/env python

from __future__ import print_function
import os
import sys

import rospy
import rospkg
pkg_path = rospkg.RosPack().get_path('darkflow_detect')
sys.path.append(pkg_path)
os.chdir(pkg_path)

from image_convert import ImageConverter
from darkflow_detect.srv import Detect, DetectResponse
from darkflow.net.build import TFNet


def handle_request(req):
    # import cv2
    # test_pic = 'test_pics/18426705_1650271368320679_1286613108_o.jpg'
    # imgcv = cv2.imread(test_pic)
    result = tfnet.return_predict(img_cvt.cv_img)

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
    if res.result == True: print()
    return res

def print_info(info):
    print('---------------------------')
    print(info['label'])
    print(info['confidence'])
    print(info['topleft'])
    print(info['bottomright'])


options = {
    "model": "cfg/yolo-test.cfg",
    "backup": "ckpt/",
    "load": -1,
    "threshold": -0.1,
    "gpu": .8
}
tfnet = TFNet(options)

rospy.init_node('object_detect', anonymous=True)
img_cvt = ImageConverter(topic="/camera/color/image_raw")

rospy.Service('detect', Detect, handle_request)
rospy.loginfo('Object detector is running.')
rospy.spin()
