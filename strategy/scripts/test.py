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
from LM_Control import *


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
        # ss.Arm.relative_move_nsa(a = 0.1)
        # exit()
        # sift_client('speed_stick')
        # exit()
        # #--------Test save_item_location() & distributioni--------#
        # #ss.stow.test_read_item_location_in_arc_pack("stow_20.json")
        # ss.stow.test_read_item_location_in_arc_pack("item_location_file.json")
        # exit()

        #--------J7 Over 180 ----------#
        ss.stow.arm_photo_pose()
        while ss.Arm.busy:
            rospy.sleep(.1)

        # case 1
        # ss.Arm.relative_rot_nsa(roll = -179.607239859, blocking = True)
        # ss.Arm.relative_xyz_base(x = 0.0475659708497, y = -0.0263450535138, z = -0.316055137686, blocking = True)

        # case 2
        # ss.Arm.relative_rot_nsa(roll = -179.607239859, blocking = True)
        # ss.Arm.relative_xyz_base(x = 0.0475659708497, y = -0.0263450535138, z = -0.316055137686, blocking = True)

        # ss.Arm.relative_move_suction(suction_angle = 90, dis =  0.02, blocking = True)
        # ss.Arm.relative_rot_pry_move_nsa(s = -0.02)
        ss.Arm.relative_move_nsa(a = 0.02)
        
        #---------Test photo pose 2 ----------#
        # ss.stow.arm_photo_pose_2()
        #ss.Arm.pub_ikCmd('ptp', (0.45, 0.02 , -0.05), (-160, 0, 0), 0)
        # ss.Arm.pub_ikCmd('ptp', (0.45, 0 , 0.1), (-180, 0, 0), 0)

        #---------Go Bin I ----------#
        #ss.stow.LM_2_Bin_Right_Arm('i')
        #ss.stow.arm_leave_tote_i_bin()

        #ss.Arm.relative_move_nsa(a = -0.23)

        exit()

        #-------Test Vision Closest----------#
        # ss.stow.test_read_item_location_in_arc_pack("stow_2_obj.json")
        # ss.stow.gen_detect_all_in_stow_list()

        # print ('request_highest_item()')

        # ss.stow.request_highest_item()
        #ss.stow.arm_photo_pose_2()
        # ss.stow.arm_photo_pose()
        # ss.stow.LM_2_tote()
        # #ss.stow.request_highest_item()
        # ss.stow.test_request_unknown_highest_item()

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
        # ss.stow.LM_2_tote()
        # ss.stow.arm_photo_pose()

        #---------LM & Arm with Bin----------#
        # gripper_suction_up()
        # #ss.stow.LM_2_Bin_No_Shift('b')
        # ss.stow.LM_2_Bin_Right_Arm('a')
        
        # ss.stow.arm_leave_tote()
        # while ss.Arm.busy:
        #     rospy.sleep(.1)

        #---------Arm Up ----------#
        #ss.stow.arm_photo_pose()
        gripper_vaccum_on()
        ss.Arm.move_2_Abs_Roll(90,blocking=True)
        ss.Arm.pub_ikCmd('ptp', (0.2, 0.00 , 0.25), (-180, 0, 0))
        
        while ss.Arm.busy:
            rospy.sleep(.1)
        #
        rospy.sleep(1)
        ss.stow.arm_leave_tote()

        
        exit()


        #---------Error Pose----------#
        # gripper_suction_up()
        # #ss.stow.LM_2_Bin_No_Shift('b')
        # #ss.stow.LM_2_Bin_Right_Arm('a')

        #
        ss.stow.LM_2_Bin_Right_Arm('d')
        
        #ss.stow.arm_leave_tote()
        #ss.Arm.relative_move_nsa(a = -0.2) 
        #ss.stow.arm_init_pose()
        ss.stow.arm_photo_pose()


        
        # while ss.Arm.busy:
        #     rospy.sleep(.1)
        # ss.Arm.relative_move_nsa(a = 0.2, blocking = True)
        # #sss.LM.pub_LM_Cmd(LM_ID_Base, 60000)
        # ss.Arm.relative_move_nsa(a = -0.2, blocking = True)


        #ss.LM.pub_LM_Cmd(LM_ID_Right, ToteLeave_Z)
        #ss.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'amnesty'))
        #ss.stow.LM_amnesty_up()
        rospy.spin()

    except rospy.ROSInterruptException:
        print "[Error] rospy error"
        