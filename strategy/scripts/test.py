#! /usr/bin/env python

import sys
from math import radians, degrees, sin, cos, tan, pi
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

import arm_task_rel
from gripper import *
from s import *




        


if __name__ == '__main__':
    rospy.init_node('s_test', disable_signals=True)

    try:
        ss = Strategy()
        #ss.stow.test_request_highest()
        
        #-------Test Vision Closest----------#
        ss.stow.test_read_item_location_in_arc_pack("stow.toteTask_00009.json")
        ss.stow.gen_detect_all_in_stow_list()
        ss.stow.request_highest_item()


        rospy.spin()

    except rospy.ROSInterruptException:
        print "[Error] rospy error"
        