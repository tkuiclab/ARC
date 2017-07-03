#!/usr/bin/env python

"""Request gqcnn to predict pose."""
# Don't use this script, because it doesn't complete

from __future__ import print_function
import sys

import rospy
from sensor_msgs.msg import Image, CameraInfo
from darkflow_detect.srv import Detect, DetectResponse
from darkflow_detect.msg import Detected
from gqcnn.srv import GQCNNGraspPlanner, GQCNNGraspPlannerResponse
from gqcnn.msg import BoundingBox, GQCNNGrasp

def obj_detect(obj_name='all'):
    """Using /detect service to detect the object."""
    rospy.wait_for_service('detect')
    detect_srv = rospy.ServiceProxy('detect', Detect)

    bbox = list()
    try:
        res = detect_srv(obj_name)
        for detected in res.detected:
            bbox.append(detected.bound_box)
    except rospy.ServiceException as e:
        rospy.logwarn('Service did not process request: {}'.format(e))
    return bbox


def predict_pose(bboxes):
    """Using /predi service to predict the pose."""
    rospy.loginfo('Service would process requests.')
    rospy.wait_for_service('predi')
    predict_srv = rospy.ServiceProxy('predi', GQCNNGraspPlanner)

    predict_success = list()
    for bbox in bboxes:
        box = BoundingBox(bbox[0], bbox[1], bbox[2], bbox[3])
        try:
            res = predict_srv(img, dep, camInfo, box)
            predict_success.append(res.grasp_success_prob)
        except rospy.ServiceException as e:
            rospy.logwarn('Service did not process request: {}'.format(e))
    return predict_success


def img_sub_cb(msg):
    """Subscribe the message for image."""
    global img
    img = msg


def dep_sub_cb(msg):
    """Subscribe the message for depth."""
    global dep
    dep = msg


def camInfo_sub_cb(msg):
    """Subscribe the message for camera infomation."""
    global camInfo
    camInfo = msg


if __name__ == '__main__':
    rospy.init_node('predict_pose', anonymous=True)

    img = Image(); dep = Image(); camInfo = CameraInfo()
    img_sub = rospy.Subscriber('camera/rgbd/image_raw', Image, img_sub_cb, queue_size=1)
    dep_sub = rospy.Subscriber('camera/depth/image_raw', Image, dep_sub_cb, queue_size=1)
    camInfo_sub = rospy.Subscriber('camera/depth/camera_info', CameraInfo, camInfo_sub_cb, queue_size=1)

    # Waiting subscriber
    rospy.sleep(2.0)

    obj_name = 'all' if len(sys.argv) < 2 else sys.argv[1]
    bboxes = obj_detect(obj_name)
    success = obj_detect(bboxes)
    print(success)
