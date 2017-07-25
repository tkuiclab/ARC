#! /usr/bin/env python

import sys
from math import radians, degrees, sin, cos, pi
from numpy import multiply
import numpy

import rospy
import roslib; #roslib.load_manifest('obj_pose')
import tf
import arm_task_rel


def safe_pose(arm):
    arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))

def init_pose(arm):
    arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-90, 0, 0))


if __name__ == '__main__':
    rospy.init_node('enable_left_arm', anonymous=True)

    rospy.sleep(0.5)

    left_arm  = arm_task_rel.ArmTask('/left_arm')
    right_arm = arm_task_rel.ArmTask()



    # left_arm.home()
    # right_arm.home()
    safe_pose(right_arm)
    init_pose(left_arm)

    while (left_arm.busy) or (right_arm.busy):
         rospy.sleep(.1)

    safe_pose(left_arm)
    init_pose(right_arm)

    while (left_arm.busy) or (right_arm.busy):
         rospy.sleep(.1)

    # safe_pose(left_arm)
    # safe_pose(right_arm)

    # while (left_arm.Arm.busy) or (right_arm.Arm.busy):
    #     rospy.sleep(.1)

    left_arm.home()
    right_arm.home()

    while (left_arm.busy) or (right_arm.busy):
         rospy.sleep(.1)