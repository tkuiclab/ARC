#! /usr/bin/env python

import sys
import roslaunch
import roslib

import rospkg
import rospy
from math import radians, degrees, sin, cos, tan, pi
from numpy import multiply
import numpy
import object_distribution
import arm_task_rel

def show_bin_data(bin):
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)

if __name__ == '__main__':
    rospy.init_node('yc_test', disable_signals=True)

    try:
        
        Arm   = arm_task_rel.ArmTask()
        # Arm.Get_Curr_Joint7_fb()
        # exit()

        # bin_a = object_distribution.BinInfo()
        
        # =========== sim stra send cmd================
        TargetBin = 'D'
        desire_cmd = [0.45, 0, 0.3, -90, 0, 0]                                # decided cmd
        LM1_cmd, LM2_cmd = 64000, 60000
        SuctionAngle = 0
        Arm.Get_Collision_Avoidance_Cmd(desire_cmd, LM1_cmd, LM2_cmd, SuctionAngle, TargetBin) # check collision
        limit = object_distribution.parse_shelf(TargetBin)                      # Get limit info

        # =============================================

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# ==========================
        # print 'Bin id is ' + str(a.block)
        # print 'min_y = ' + str(a.min_y)
        # print 'max_y = ' + str(a.max_y)
        # print 'min_z = ' + str(a.min_z)
        # print 'max_z = ' + str(a.max_z)

        # a = object_distribution.parse_shelf(3)