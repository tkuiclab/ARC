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
from sift.srv import *


def safe_pose(arm):
    arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
    rospy.sleep(.5)
    while arm.busy:
        rospy.sleep(.1)

# def test_SIFT():


def sift_client(obj_name):
    rospy.wait_for_service('/sift_server')
    try:
        client = rospy.ServiceProxy(
            '/sift_server',
             sift
        )

        res = client(obj_name)

        print('xmin =' + str(res.xmin))
        print('xmax =' + str(res.xmax))
        print('xmax =' + str(res.ymin))
        print('xmax =' + str(res.ymax))
        

       
    except rospy.ServiceException, e:
        print "Service call (Vacuum) failed: %s" % e

if __name__ == '__main__':
    rospy.init_node('s_test', disable_signals=True)

    try:
        ss = Strategy()

        #safe_pose(ss.Arm)
        sift_client('speed_stick')
        exit()
        #--------Test save_item_location() & distributioni--------#
        #ss.stow.test_read_item_location_in_arc_pack("stow_20.json")
        ss.stow.test_read_item_location_in_arc_pack("item_location_file.json")
        exit()


        #-------Test Vision Closest----------#
        ss.stow.test_read_item_location_in_arc_pack("stow_2_obj.json")
        ss.stow.gen_detect_all_in_stow_list()

        print ('request_highest_item()')

        ss.stow.request_highest_item()


        # ----- Test Photo Pose ------#
        # ss.stow.LM_2_tote()
        # ss.stow.arm_photo_pose()

        
        #ss.stow.test_request_highest()


        #ss.stow.arm_photo_pose()

        
        # ss.Arm.relative_move_nsa(a = -0.2)
        
        #ss.stow.LM_2_Bin('a')
        # ----- Pick all Unknown Highest------#
        # s.stow.test_read_item_location_in_arc_pack("stow.toteTask_00021.json") #any file is ok
        # s.stow.test_all_unknown_2_amnesty()
        # gripper_vaccum_off()
        # s.start() 
        # s.stow_run()


        # ----- Test Photo Pose ------#
        ss.stow.LM_2_tote()
        ss.stow.arm_photo_pose()

        #---------LM & Arm with Bin----------#
        gripper_suction_up()
        #ss.stow.LM_2_Bin_No_Shift('b')
        ss.stow.LM_2_Bin_Right_Arm('d')
        
        ss.stow.arm_leave_tote()
        while ss.Arm.busy:
            rospy.sleep(.1)
        # ss.Arm.relative_move_nsa(a = 0.2)
        #sss.LM.pub_LM_Cmd(LM_ID_Base, 60000)


        
        rospy.spin()

    except rospy.ROSInterruptException:
        print "[Error] rospy error"
        