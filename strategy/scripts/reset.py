#! /usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos, pi
from numpy import multiply
import numpy

import rospy
import roslib; #roslib.load_manifest('obj_pose')
import tf


import actionlib
from std_msgs.msg import String, Float64, ByteMultiArray

import obj_pose.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist


import json

import arm_task_rel
from gripper import *


obj_dis = 0.1

class T2O:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        #self.__set_pubSub()
        self.Arm 			= arm_task_rel.ArmTask()
        rospy.on_shutdown(self.stop_task)

    def stop_task(self):
        """Stop task running."""
        self.Arm.stop_task()

    def safe_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)

if __name__ == '__main__':

    rospy.init_node('robot_reset', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    rospy.loginfo('robot_reset Ready')

    # -------Back 2 home------#.
    task.safe_pose()
    task.Arm.home()

    while task.Arm.busy:
        rospy.sleep(.1)