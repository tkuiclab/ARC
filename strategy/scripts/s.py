#!/usr/bin/env python

# pylint: disable = invalid-name
# pylint: disable = C0326, C0121, C0301
# pylint: disable = W0105, C0303, W0312

import math
import threading
import time

import actionlib
import roslaunch
import roslib

import rospkg
import rospy
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String
from strategy.srv import *

import arm_task_rel
import LM_Control
#import task_parser
from task_parser import *
from config import *
from gripper import *
from pick_task import PickTask
from stow_task import StowTask

import json

TaskType_None = 0
TaskType_Pick = 1
TaskType_Stow = 2



class Strategy(threading.Thread):
    """ description """
    def __init__(self):
        threading.Thread.__init__(self)
        rospy.on_shutdown(self.shutdown)
        rospy.Service('/task', Task, self.task_cb)
        self.info_pub = rospy.Publisher('/stratege/info', String, queue_size=10)
        
        # === Initialize All Var === 
        #self.Arm 			= arm_task_rel.ArmTask('/left_arm/robotis')
        #self.right_Arm 			= arm_task_rel.ArmTask('/right_arm/robotis')
        self.Arm 			= arm_task_rel.ArmTask()
        self.LM  			= LM_Control.CLM_Control()

        self.pick = PickTask(self.Arm, self.LM)		
        self.stow = StowTask(self.Arm, self.LM)

        self.stop_robot = False

        self.run_task_type = TaskType_None

        rospy.sleep(0.3)
        rospy.loginfo("Strategy Ready!!")
        
    def shutdown(self):
        """ description """
        self.stop_robot = True
        rospy.loginfo("Strategy Exit & Stop Robot")

    def safe_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (-180, 0, 0))
        while self.Arm.busy:
            rospy.sleep(.1)

    def stow_run(self):
        if self.stow.is_ready() :
            rospy.loginfo('Stow Task Running')
            self.run_task_type = TaskType_Stow
            self.stow.run()  
        else:
            rospy.logwarn('Stow Task Not Ready!!')


    def task_cb(self,req):
        """ description """
        task_name = req.task_name
        rospy.loginfo("task_name = " + task_name)
        if task_name.lower() == 'stow':
            self.run_task_type = TaskType_Stow
        elif task_name.lower() == 'pick_json_item_location':
            self.pick.save_item_location(req.task_json)
        elif task_name.lower() == 'pick_json_order':
            self.pick.save_order(req.task_json)
        elif task_name.lower() == 'pick_run':
            if self.pick.is_ready() :
                rospy.loginfo('Pick Task Running')
                self.run_task_type = TaskType_Pick
                self.pick.run()  
            else:
                rospy.logwarn('Pick Task Not Ready!!')
        elif task_name.lower() == 'stow_run':
            self.stow_run()
        elif task_name.lower() == 'stow_json_item_location':
            self.stow.save_item_location(req.task_json)
        else:
            print 'Error Task Name (Please input pick or stow)'
        
        r = TaskResponse()
        r.success = True
        r.msg = " GET " + task_name.upper() +  "command"
        return r

    def run(self):
        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.stop_robot == True :
                return
            
            if  self.run_task_type != TaskType_None:
                if self.run_task_type == TaskType_Pick:
                    self.pick.pick_core()
                    #info_json = self.pick.get_info()
                elif self.run_task_type == TaskType_Stow:
                    self.stow.stow_core()
                    #info_json = self.stow.get_info()
                    
                #self.info_pub.publish(json.dumps(info_json))
            
            rate.sleep()

    def test_go_bin_LM(self, bin):
        rospy.sleep(0.5)
        print 'test_go_bin_LM bin -> ' +bin  
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin ))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin ))

    def test_go_box(self, box):
        rospy.sleep(0.5)
        print 'test_go_box_LM box -> ' + box  
        self.LM.pub_LM_Cmd(2, GetShift('Box', 'x', box ))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Box', 'z', box ))
        rospy.sleep(0.5)	

    def test_publish_info(self):
        info_json = {'info': "(GoBox) Vaccum Disable - [Success]", 
                'item': 'mesh_cup', 
                'bin': 'E',
                'box': '1A5'
                }
        self.info_pub.publish(json.dumps(info_json))

    def arm_go_init_pose(self):
        rospy.loginfo('Arm go init pose')
        self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.22), (0, 0, 0))

    def arm_bin_photo(self):
        self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.34), (0, 0, 0))


if __name__ == '__main__':
    rospy.init_node('strategy', disable_signals=True)

    try:
        s = Strategy()

        bin_id = 'e'
        s.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin_id))
        rospy.sleep(0.3)
        s.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin_id))
        # s.LM.pub_LM_Cmd(1, 20000)
        # rospy.sleep(0.3)
        # s.LM.Show_LM_FB()

        # s.LM.rel_move_LM('left', 5)
        # s.LM.rel_move_LM('left', -5)
        # rospy.sleep(0.3)
        # s.LM.rel_move_LM('base', 5)
        # s.LM.rel_move_LM('base', -5)

        # s.stow.test_read_item_location_in_arc_pack("stow_test.json")
        # #s.stow.test_read_item_location_in_arc_pack("stow_1_obj.json")
        
        # #s.safe_pose()
        # gripper_vaccum_off()
        # s.start() 
        # s.stow_run()


        # # s.Arm.pub_ikCmd('ptp', (0.25, 0.0 , 0.2), (-90, 0, 0) )
        # # gripper_vaccum_on()


        # s.stow.arm_photo_pose()
        
        #s.stow.test_read_item_location_in_arc_pack("stow.toteTask_00021.json")
        #s.stow.test_read_item_location_in_arc_pack("stow.toteTask_00005.json")

        # s.stow.test_read_item_location_in_arc_pack("stow.toteTask_00009.json")
        
        # s.stow.gen_detect_all_in_stow_list()
        # print "detect_all_in_stow_list[] -> " + str(s.stow.detect_all_in_stow_list)

        # rospy.sleep(10)
        
        # s.stow.test_2_stow_fail()

        #print str(s.stow.detect_all_in_stow_list)

        # gripper_vaccum_off()
        # #s.stow.test_read_item_location_in_arc_pack("stow.toteTask_00021.json")
        # s.stow.test_read_item_location_in_arc_pack("stow_test.json")
        # rospy.sleep(0.3)
        # s.stow_run()

        # write_PickInfo_2_JSON()

        # ========== TEST Bin Put in===========
        # s.test_go_bin_LM('h')

        
        # #arm_leave_tote
        # s.Arm.pub_ikCmd('ptp', (0.25, 0.0 , 0.2), (-90, 0, 0) )
        # gripper_suction_up()
        #s.Arm.relative_move_nsa(a = -0.35) 

        # ========== TEST ===========
        #s.test_go_bin_LM('e')

        #s.safe_pose()

        # ========== TEST ===========
        #s.test_go_bin_LM('e')

        #s.safe_pose()

        # ============ rel motion test area start ============
        # s.Arm.relative_move_nsa(n = 0.04)  #n = move_cam_y
        # s.Arm.relative_move_nsa(s = -0.06) #s= move_cam_x
        # s.Arm.relative_move_nsa(a = 0.1)   #a= move_cam_z

        # ========================= rel motion test area start =============================
        # s.Arm.pub_ikCmd('ptp', (0.3, 0 , 0.2), (-180, 0, 0) )
        # dis = -0.05
        # rot = 20
        # s.Arm.pub_ikCmd('ptp', (0.3, -0.05 , 0.2), (-150, -40, 0) )
        # s.Arm.relative_rot_nsa(roll = rot)
        # s.Arm.relative_move_nsa(s = dis)

        # s.Arm.pub_ikCmd('ptp', (0.3, -0.05 , 0.2), (-150, -40, 0) )
        # s.Arm.relative_rot_pry_move_nsa(s = dis, roll = rot)



        # s.Arm.relative_move_nsa_rot_pry(a = dis, pitch = rot)
        # s.test_relative_move_nsa(0.05)
        # s.test_relative_rot_nsa(10)
        # s.test_relative_xyz_base(0.05)
        # s.test_relative_move_nsa_rot_pry(dis = 0.05, rot = 10)
        # s.test_relative_move_xyz_rot_pry(dis = 0.05, rot = 10)

        # ========================= rel motion test area over ==============================

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
