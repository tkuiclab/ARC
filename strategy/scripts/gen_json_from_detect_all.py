#! /usr/bin/env python
# pylint: disable = invalid-name
# pylint: disable = C0326, C0121, C0301
# pylint: disable = W0105, C0303, W0312
"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos, pi
from numpy import multiply
import numpy

import rospy
import roslib; #roslib.load_manifest('obj_pose')
import tf


import actionlib
from std_msgs.msg import String, Float64

import obj_pose.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import json

from darkflow_detect.srv import Detect, DetectResponse
from darkflow_detect.msg import Detected

def request_all():
    detect_client = rospy.ServiceProxy('/detect', Detect)
    res = detect_client("all")
    item_content = []

     
    for d_item in res.detected :
        print('"' + d_item.object_name + '",')
    

    
if __name__ == '__main__':

    rospy.init_node('gen_json_from_detect_all', anonymous=True)


    rospy.sleep(0.5)
    
    request_all()