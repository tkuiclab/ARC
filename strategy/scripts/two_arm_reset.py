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
    arm.pub_ikCmd('ptp', (0.25, 0.00 , 0.32), (-145, 0, 0))

def init_pose(arm):
    arm.pub_ikCmd('ptp', (0.35, 0.00 , 0.3), (-180, 0, 0))

def LeftArm_pose(arm):
    arm.pub_ikCmd('ptp', (0.4, -0.13 , 0.00), (-180, 0, 0), fai = -50)

def RightArm_pose(arm):
    arm.pub_ikCmd('ptp', (0.4, 0.13 , 0.00), (-180, 0, 0), fai = 50)

def test_fn():
    # description: Test is the robot can work?

    # back hom first
    left_arm.home()
    right_arm.home()
    while (left_arm.busy) or (right_arm.busy):
         rospy.sleep(.1)
    

    # go to init pose
    safe_pose(left_arm)
    init_pose(right_arm)
    while (left_arm.busy) or (right_arm.busy):
         rospy.sleep(.1)


    # back home again
    # left_arm.home()
    # right_arm.home()
    # while (left_arm.busy) or (right_arm.busy):
    #      rospy.sleep(.1)


if __name__ == '__main__':
    rospy.init_node('enable_left_arm', anonymous=True)
    rospy.sleep(0.5)
    left_arm  = arm_task_rel.ArmTask('/left_arm')
    right_arm = arm_task_rel.ArmTask()

    left_arm.home()
    right_arm.home()

    # test_fn()
