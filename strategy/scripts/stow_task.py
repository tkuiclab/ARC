#!/usr/bin/env python
import math
import threading
import time
import numpy
from math import radians, degrees, sin, cos, pi

import actionlib
import roslaunch
import roslib; roslib.load_manifest('obj_pose')
import rospkg
import rospy
from darkflow_detect.srv import Detect, DetectResponse
from darkflow_detect.msg import Detected

import obj_pose.msg
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String, ByteMultiArray
from strategy.srv import *

import arm_task_rel
import LM_Control
from LM_Control import LM_ID_Base ,LM_ID_Right, LM_ID_Left
from task_parser import *
from config import *
from gripper import *
import get_obj_info


# Define State
WaitTask = 1		# Wait Task
#Init_Pos = 4		# Make robot arm go to the initial pos
LM2Bin = 5		# Make robot arm go to the specify bin
WaitRobot = 6		# wait for robot complete task
ArmLeaveBin = 8		# Make robot arm leave bin
FinishOne = 9
ArmDown_And_Vaccum = 11
PlaceObj = 12
Recover2InitPos = 15
LM2Tote = 16
ArmLeaveTote = 18
Shift2Stow = 20
ArmPutInBin = 21
GripperOff = 22
LM2Amnesty = 24
PhotoPose = 26
VisionProcess = 27
WaitVision = 28
Arm2ObjUp = 30
LM_LeaveTote = 31
EndTask  = 34


LM_Down_2_Amnesty =  40
AmnestyGripperOff =  41
Amnesty_End = 42

LM2Amnesty_Up   = 43
LM2Amnesty_Down = 44

Mode_KnownProcess = 1
Mode_UnknownProcess = 2


LM_UP_SHIFT = 2000


# For checking vacuum function
check_next_states = [
    #LM_LeaveTote,
    ArmLeaveTote,
    LM2Bin,
    ArmPutInBin,
    LM2Amnesty_Up
]

obj_dis = 0.01


Arm_Photo_Index_Max  = 2


pose_1_vision_limit_ary = [-0.13, 0.13, -0.15,  0.3, 0.35, 0.6]
pose_2_vision_limit_ary = [-0.13, 0.13, -0.15,  0.3, 0.35, 0.6]



class StowTask:

#--------------------Init & ROS--------------------#

    """ Init  """

    def __init__(self, i_arm, i_LM):

        self.Arm = i_arm
        self.LM = i_LM
        self.ros_init()

        self.var_init()

        rospy.loginfo('StowTask::init()')

    def ros_init(self):
        self.__if_suck_sub = rospy.Subscriber(
            '/IfSuck',
            ByteMultiArray,
            self.ifsuck_cb,
            queue_size=1
        )

        self.obj_pose_client = actionlib.SimpleActionClient(
            "/obj_pose", obj_pose.msg.ObjectPoseAction)

    def var_init(self):
        # === Initialize State ===
        self.state = PhotoPose #LM2Tote
        self.next_state = WaitTask
        self.Is_ArmBusy = False
        self.Is_LMBusy = False
        self.Last_LM_Busy = False
        self.Is_LMArrive = True
        self.Last_LMArrive = True
        self.Is_BaseShiftOK = False
        self.Is_ArmMoveOK = False

        self.BinCnt = 0
        self.Box = 'a'
        self.Tote = 'a'

        # all item from item_location file, get from make_stow_list()
        # remove item if success(->stow_success) or fail (->stow_fail)
        #
        self.stow_list = []
        self.stow_fail = []
        self.stow_fail_2 = []
        self.stow_success = []
        self.stow_amnesty = []
        
        self.arm_photo_index  = 1

        self.use_stow_list = self.stow_list

        self.mode = Mode_KnownProcess


    def ifsuck_cb(self, res):
        self.suck_ary = res.data

    @property
    def suck_num(self):
        return sum(self.suck_ary)


#--------------------Control Area--------------------#
    def LM_2_tote(self):
        self.info = "LM_2_tote()"
        print self.info

        self.Is_BaseShiftOK = False
        self.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'tote'))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'tote'))

    def LM_2_amnesty(self):
        self.info = "(GoTote) Shift 2 Amnesty tote "
        print self.info

        self.Is_BaseShiftOK = False
        self.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'amnesty'))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'amnesty'))

    def LM_amnesty_down(self):
        self.info = "(GoTote) Down to amnesty "
        print self.info

        self.Is_BaseShiftOK = False
        self.LM.pub_LM_Cmd(2, GetShift('Tote', 'x', 'amnesty'))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Tote', 'z', 'amnesty'))

    def LM_amnesty_up(self):
        self.info = "___LM_amnesty_up() "
        print self.info

        self.Is_BaseShiftOK = False
        self.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'amnesty'))
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, ToteLeave_Z)

    # def LM_up_from_amnesty(self):

    #     self.Is_BaseShiftOK = False
    #     self.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'amnesty'))
    #     rospy.sleep(0.3)
    #     self.LM.pub_LM_Cmd(1, GetShift('Tote', 'z', 'tote'))

    def LM_2_Bin(self, i_bin):
        # self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', i_bin ) + _LEFT_SHIFT)
        # rospy.sleep(0.3)
        # self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', i_bin)  + _DOWN_SHIFT)
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin ) + LM_Right_Arm_Shift)
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin ) - LM_UP_SHIFT)


    def LM_2_Bin_No_Shift(self, i_bin):
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', i_bin ) )
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', i_bin)  )
        
    def LM_2_Bin_Right_Arm(self, i_bin):
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', i_bin ) + LM_Right_Arm_Shift )
        rospy.sleep(0.3)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', i_bin)  - LM_UP_SHIFT)

    

    def arm_photo_pose(self):
        #self.Arm.pub_ikCmd('ptp', (0.4, 0.00, 0.15), (-180, 0, 0))
        #self.Arm.pub_ikCmd('ptp', (0.5, 0.00, 0.15), (-180, 0, 0))
        self.Arm.pub_ikCmd('ptp', (0.45, 0.00 , 0.25), (-180, 0, 0))

        self.vision_limit_ary = pose_1_vision_limit_ary
        self.tool_shot_deg = 0
     
    def arm_photo_pose_2(self): #arm_photo_pose_2(self):
        #self.Arm.pub_ikCmd('ptp', (0.5, 0.00, 0.15), (-180, 0, 0))
        self.Arm.pub_ikCmd('ptp', (0.35, 0.00 , 0.20), (180, 180, 0))
        self.vision_limit_ary = pose_2_vision_limit_ary
        self.tool_shot_deg = 180

    def arm_photo_pose_3(self):
        self.Arm.pub_ikCmd('ptp', (0.4, 0.00 , 0.25), (180, 180, 0))

    def arm_leave_tote(self):
        self.Arm.pub_ikCmd('ptp', (0.25, 0.0 , 0.2), (-90, 0, 0) )

    def arm_leave_tote_i_bin(self):
        self.Arm.pub_ikCmd('ptp', (0.25, -0.02 , 0.2), (-90, 0, 0) )


    def arm_leave_tote_safe(self):
        self.Arm.move_2_Abs_Roll(90,blocking=True)
        self.Arm.pub_ikCmd('ptp', (0.2, 0.00 , 0.25), (-180, 0, 0))

    def arm_init_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (-90, 0, 0) )

    def tool_2_obj(self, obj_pose, norm, shot_deg = 0): #STOW
        p = obj_pose
        a = p.angular
        l = p.linear

        # rospy.loginfo("object_pose")
        # rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        # rospy.loginfo("(roll,pitch,yaw)= (" 
        #                 + str(numpy.rad2deg(a.x)) + ", " 
        #                 + str(numpy.rad2deg(a.y)) + ", " 
        #                 + str(numpy.rad2deg(a.z)) + ")" )
        # rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        if (l.x * norm.x > 0) and (abs(norm.x) > abs(norm.y)) and (abs(norm.x) > abs(norm.z)) :
            print("#################\nInvert Norm.x\n#################")
            norm.x = norm.x*-1
            norm.y = norm.y*-1
            norm.z = norm.z*-1
            print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        old_y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        old_r = 90 - (numpy.rad2deg(a.x) + 180)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        # print("(old_y, old_r)= (" + str(old_y) + ", " + str(old_r) + ")")
        # print("(y, r)= (" + str(y) + ", " + str(r) + ")")
        
        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y_4_tote) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        #rospy.loginfo("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]

        # rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        # rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        # rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")

        # #----------------Rotation---------------_#
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        self.Arm.relative_rot_nsa(roll = y, blocking = True)
        gripper_suction_deg(r)

        # print('=====')
        # print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        # print('self.Arm.gripper_suction_deg('+str(r)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_y*-1)+', y = '+str(real_move_x)+', z = '+str(real_move_z*-1)+')')
        self.Arm.relative_xyz_base(x = real_move_y*-1, y = real_move_x, z = real_move_z*-1, blocking = False)

        
        # if real_move_z > 0.15:
        #     self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'tote')+15000)
        #     self.Arm.relative_xyz_base(x = real_move_y*-1, y = real_move_x, z = (real_move_z-0.15)*-1, blocking = False)
        # else:
        #      self.Arm.relative_xyz_base(x = real_move_y*-1, y = real_move_x, z = real_move_z*-1, blocking = False)


        self.gripper_roll = r

        #rospy.loginfo('Move Angle Finish')
    
    
    def update_status(self):
        """ update ARM & LM status  """
        self.Last_LM_Busy 	= self.Is_LMBusy
        self.Last_LMArrive 	= self.Is_LMArrive
        self.Is_ArmBusy 	= self.Arm.busy
        self.Is_LMBusy  	= self.LM.IsBusy
        self.Is_LMArrive	= self.LM.IsArrive

#--------------------Vision Area & obj_pose_cb--------------------#	

    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

    def arm_chage_side_and_state(self):
        
        if self.arm_photo_index < Arm_Photo_Index_Max:
            self.arm_photo_index = self.arm_photo_index + 1
            self.state = FinishOne

            self.mlog('>>>>>>>>>>>> arm_chage_side  <<<<<<<<<<<<< self.arm_photo_index = ' + str(self.arm_photo_index))

        else:
            # self.arm_photo_index >= Arm_Photo_Index_Max
            if self.mode == Mode_KnownProcess:
                self.mode = Mode_UnknownProcess
                self.state = PhotoPose #LM2Tote
                self.arm_photo_index = 1

                self.mlog('--------------- Mode_UnknownProcess--------------')

            else:

                self.mode == Mode_KnownProcess
                self.arm_photo_index = 1
                
                # if self.use_stow_list != self.stow_fail:
                #     self.mlog('==================Stow Fail==================')

                #     self.state = FinishOne
                #     self.use_stow_list = self.stow_fail
                #     self.arm_photo_index = 1
                # else:
                #     self.state = EndTask
    
    def unknown_change_2_known(self):
        self.mlog('---------------Change back 2 Mode_KnownProcess--------------')
        self.mode = Mode_KnownProcess
        self.arm_photo_index = 1
        self.use_stow_list = self.stow_list
    

    def get_detect_all_list(self):
        detect_client = rospy.ServiceProxy('/detect', Detect)
        res = detect_client("all")
        return res.detected

    def request_highest_item(self):
        print 'in request_highest_item + self.detect_all_in_stow_list=' + str(self.detect_all_in_stow_list)

        if len(self.detect_all_in_stow_list) > 0:
            goal = obj_pose.msg.ObjectPoseGoal(
                object_name = "<Closest>",
                object_list = self.detect_all_in_stow_list,
                #limit_ary =[-0.3, 0.3, 0,  0.4, 0.3, 0.6]
                #limit_ary =[-0.14, 0.14, 0,  0.4, 0.3, 0.6]
                limit_ary = self.vision_limit_ary
            )

            self.obj_pose_client.send_goal(
                    goal,
                    feedback_cb = self.obj_pose_feedback_cb, 
                    done_cb=self.obj_pose_done_cb )
            return True
        else:
            #self.arm_chage_side_and_state()
            rospy.logwarn("len( detect_all_in_stow_list) <=0")
            return False

    def obj_pose_done_cb(self, state, result):
        self.obj_pose = result.object_pose
        self.norm = result.norm
        l = result.object_pose.linear
        if result.success == False or (l.z < 0) or (l.x == 0 and l.y == 0 and l.z==0):
            rospy.logwarn('ROI Fail!!')
            
            self.arm_chage_side_and_state()
            return
        else:
            print '-----------obj_pose_done_cb---[highest]---<< ' + result.object_name +' >>--------------'
            self.obj_pose = result.object_pose
            self.now_stow_info = self.get_stow_info_by_name(result.object_name)
            #self.set_stow_method(self.now_stow_info)
            rospy.loginfo("now_stow_info = " + self.now_stow_info.item)

            if(self.now_stow_info== None):
                self.state = FinishOne
            else :
                self.state = Arm2ObjUp

        self.mode = Mode_KnownProcess

    

    def request_unknown_highest_item(self):
        fix_ary = [0.03, -0.03, 0.02, -0.02, -0.05,0]

        new_limit_ary = []
        for i in range(len(fix_ary)):
            #print('i='+str(i))
            new_limit_ary.append(self.vision_limit_ary[i] +fix_ary[i])


        goal = obj_pose.msg.ObjectPoseGoal(
            object_name = "<Unknown_Closest>",
            #limit_ary =[-0.14, 0.14, 0,  0.4, 0.3, 0.6]
            limit_ary = new_limit_ary #self.vision_limit_ary
        )
        self.obj_pose_client.send_goal(
                goal,
                feedback_cb = self.obj_pose_feedback_cb, 
                done_cb=self.unknown_obj_pose_done_cb )

    def unknown_obj_pose_done_cb(self, state, result):
        print "unknown_obj_pose_done_cb!!!!!!!!!!"
        self.obj_pose = result.object_pose
        self.norm = result.norm
        l = result.object_pose.linear
        if result.success == False or (l.z < 0) or (l.x == 0 and l.y == 0 and l.z==0):
            rospy.logwarn('Unknown Highest Fail!!')
            self.arm_chage_side_and_state()
            return
        else:
            self.mode = Mode_UnknownProcess
            self.state = Arm2ObjUp

#--------------------Strategy Area--------------------#
    def run(self):
        self.stow_id = 0
        self.stow_get_one()
        self.state 	= PhotoPose #LM2Tote 
        self.use_stow_list = self.stow_list

    def is_ready(self):
        if self.use_stow_list != None:
            return True
        else: 
            return False

    def stow_get_one(self):
        """ Get one pick task """
        """ Set: self.Bin, self.pick_id, self.Box """
        # if self.stow_id >= len(self.stow_list):
        # 	return False
        
        # self.now_stow_info = self.stow_list[self.stow_id]
        
        # self.stow_id = self.stow_id + 1
        
        # return True
        if len(self.stow_list):
            return True
        else:
            if self.use_stow_list != self.stow_fail:
                self.state = FinishOne
                self.use_stow_list = self.stow_fail
                return True
            else:
                self.state = EndTask
                return False


    def task_finish(self):
        """ All Tasks Finish """
        self.state			= WaitTask
        self.next_state 	= WaitTask
        self.Is_ArmBusy 	= False
        self.Is_LMBusy		= False
        self.Last_LM_Busy 	= False
        self.Is_LMArrive   	= True
        self.Last_LMArrive 	= True
        self.Is_BaseShiftOK = False
        self.BinCnt 		= 0

    def save_item_location(self,item_location_file):
        """ Note: Get stow_list """
        rospy.loginfo("[Stow] Save item_location")

        self.item_location_file = item_location_file
        self.stow_list = make_stow_list(item_location_file)		
        self.use_stow_list = self.stow_list
    
    def dump_stow_list_2_item_location_file(self):
        """Update item location file and save."""
        # Pare Json
        item_location_json = json.loads(self.item_location_file)
        rospy.loginfo('Save stow_success to item location file')
        # Remove item in bin
        for t_stow_info in self.stow_success:
            for bin in item_location_json['bins']:
                if bin['bin_id'].lower() == t_stow_info.to_bin:
                    print 'append ' + t_stow_info.item
                    bin['contents'].append(t_stow_info.item)
                    break
            item_location_json['tote']['contents'].remove(t_stow_info.item)
       
        # Savie item location file
        write_item_location(item_location_json, filetype='Stow')
    
    # def get_info(self):
    # 	if self.now_stow_info
    # 	info_json = {'info': self.info, 
    # 			'item': self.now_stow_info.item, 
    # 			'bin': self.now_stow_info.to_bin
    # 			}
        
    # 	return info_json

    # Need Var: self.use_stow_list 
    # Output: stow_info
    def get_stow_info_by_name(self, name):
        for stow_info in self.use_stow_list: #self.stow_list:
            if name == stow_info.item:
                return stow_info

        rospy.logwarn("get_stow_info_by_name() say Cannot stow_info in stow_list by name= "+ name)
        return None		
        
    """ Get detec_all from /detect and check it in stow_list"""
    # Need Var: self.use_stow_list 
    # Output Var: self.detect_all_in_stow_list  (string list)
    def gen_detect_all_in_stow_list(self):
        detect_all_list = self.get_detect_all_list()
        self.detect_all_in_stow_list = []

        # print "detect_all_list[] -> "
        # print str(detect_all_list) 

        # print "self.stow_list[] -> "
        # print str(self.stow_list) 


        # print "self.use_stow_list[] -> "
        # print str(self.use_stow_list) 

        # self.print_stow_list_item(self.use_stow_list, "use_stow_list")


        for d_item in detect_all_list :
            for s_item in self.use_stow_list:
                if d_item.object_name == s_item.item:
                    self.detect_all_in_stow_list.append(d_item.object_name)
    
    # def set_stow_method(self, i_stow_info):
    #     now_num = len(self.stow_success) 
    #     if now_num < 3:
    #         i_stow_info.to_bin = 'h'
    #     elif now_num < 6:
    #         i_stow_info.to_bin = 'e'
    #     elif now_num < 9:
    #         i_stow_info.to_bin = 'b'
    #     elif now_num < 12:
    #         i_stow_info.to_bin = 'i'
    #     elif now_num < 15:
    #         i_stow_info.to_bin = 'f'
        
    #     #i_stow_info.gripper_down = True
    #     i_stow_info.gripper_down = False

    # input: n_s = next_state
    # output: Bool
    def check_vaccum_by_next_state(self, n_s):
        """Checking status of vacuum is hold."""
        # These state need to check vacuum is hold
        if n_s in check_next_states:
            self.info = "(Check) Status of Suction: {}".format(self.suck_num)
            print(self.info)
            #return self.suck_num > 1
            return self.suck_num > 0

        # Other state do not
        #return False
        return True
        
        #if n_s == LM_LeaveTote or n_s == ArmLeaveTote or \
        # if n_s == ArmLeaveTote or \
        # n_s == LM2Bin or n_s == ArmPutInBin:
        #     if self.suck_num == 0:
        #         return False
        #     else:
        #         return True
        # else:
        #     return True
                
#--------------------Stow_Core--------------------#	

    def stow_core(self):
        self.update_status()

        if self.state == WaitTask:
            return
        if self.state == WaitVision:
            return
        
        
        # elif self.state == Init_Pos:       # step 1
        #     self.info = "(GoTote) Arm Init_Pos "  
        #     print self.info

        #     self.Arm.pub_ikCmd('ptp', (0.30, 0.0 , 0.22), (0, 0, 0)  )
            

        #     self.next_state = LM2Tote
        #     self.state 		= WaitRobot
        #     # self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
            
        #     return

        elif self.state == PhotoPose:    
            self.info = "(PhotoPose)"  
            print self.info



            if self.arm_photo_index == 1:
                print '--------Go arm_photo_pose--------'
                self.arm_photo_pose()
            elif self.arm_photo_index == 2:
                print '--------Go arm_photo_pose 2--------'
                self.arm_photo_pose_2()
            elif self.arm_photo_index == 3:
                print '--------Go arm_photo_pose 3--------'
                self.arm_photo_pose_3()
            
            gripper_suction_up()

            #self.next_state = VisionProcess
            self.state 		= WaitRobot
            self.next_state = LM2Tote
            
            
            return

        elif self.state == LM2Tote: 
            self.info = "(LM2Tote)"  
            
            print self.info

            self.LM_2_tote()
            #gripper_suction_up()
            gripper_vaccum_off() 

            self.state 			= WaitRobot
            self.next_state 	= VisionProcess 
            return


        elif self.state == VisionProcess:

            '''    
            self.info = "(Vision) Request Object_Pose with "  + self.now_stow_info.item
            print self.info
            goal = obj_pose.msg.ObjectPoseGoal(self.now_stow_info.item)
            self.obj_pose_client.send_goal(
                    goal,
                    feedback_cb = self.obj_pose_feedback_cb, 
                    done_cb=self.obj_pose_done_cb )
            '''

            if self.mode == Mode_KnownProcess:
                self.info = "(VisionProcess) Mode_KnownProcess"
                print self.info
                
                self.gen_detect_all_in_stow_list()
                print "\tdetect_all_in_stow_list[] -> " + str(self.detect_all_in_stow_list)
                self.info = "(Vision) Request highest"
                print self.info
                
                if self.request_highest_item() == True:
                    self.state 		= WaitVision
                else:
                    self.arm_chage_side_and_state()
            elif self.mode == Mode_UnknownProcess:
                self.info = "(VisionProcess) Mode_UnknownProcess"
                print self.info

                self.request_unknown_highest_item()

                self.state 		= WaitVision
            else:
                rospy.logerr("self.state == VisionProcess: Fail self.mode = " + self.mode )
                self.state 		= PhotoPose

                
            
            return
        

        elif self.state == Arm2ObjUp:
            self.info = "(Arm2ObjUp)"  
            print self.info

            # if self.arm_photo_index == 1:
            #     self.tool_2_obj(self.obj_pose, self.norm, self.tool_shot_deg)
            # elif self.arm_photo_index == 2:
            #     self.tool_2_obj(self.obj_pose, self.norm, self.tool_shot_deg)
            self.tool_2_obj(self.obj_pose, self.norm, self.tool_shot_deg)

            self.next_state = ArmDown_And_Vaccum 
            self.state 		= WaitRobot

            return

        elif self.state == ArmDown_And_Vaccum:	      # Enable Vacuum 
            
            if self.mode == Mode_KnownProcess:
                self.info = "(ArmDown_And_Vaccum) Pick" + self.now_stow_info.item
            else:
                self.info = "(ArmDown_And_Vaccum)"

            print self.info

            gripper_vaccum_on()

            #self.Arm.relative_move_suction('ptp', self.gripper_roll, obj_dis +0.02,blocking= True )
            self.Arm.relative_move_suction('ptp', self.gripper_roll, obj_dis +0.02,blocking= True )
            #print("self.Arm.relative_move_suction('ptp', "+str(self.gripper_roll)+", obj_dis +0.02)")


            if self.suck_num == 0:
                self.Arm.relative_move_suction('ptp', self.gripper_roll, 0.01,blocking= True )

            if self.suck_num == 0:
                self.Arm.relative_move_suction('ptp', self.gripper_roll, 0.01,blocking= True )

            if self.suck_num == 0:
                self.Arm.relative_move_suction('ptp', self.gripper_roll, 0.01,blocking= True )

            if self.suck_num == 0:
                self.Arm.relative_move_suction('ptp', self.gripper_roll, 0.01,blocking= True )

            if self.suck_num == 0:
                self.Arm.relative_move_suction('ptp', self.gripper_roll, 0.01,blocking= True )


            self.next_state = LM_LeaveTote
            self.state = WaitRobot

            return

        elif self.state == LM_LeaveTote:
            self.info = "(LM_LeaveTote)"
            print self.info
            
            self.Is_BaseShiftOK = False
            
            #self.LM.rel_move_LM('right',35)
            #self.next_state = ArmLeaveTote  #LM2Bin #ArmLeaveTote
            if self.mode == Mode_KnownProcess:
                self.LM.pub_LM_Cmd(LM_ID_Right, ToteLeave_Z)
                self.next_state = ArmLeaveTote
            else:
                self.LM.pub_LM_Cmd(LM_ID_Right, ToteLeave_Z_Amnesty)
                self.next_state = LM2Amnesty_Up
                #self.next_state = WaitTask

            self.state 		= WaitRobot
            return



        elif self.state == ArmLeaveTote:
            self.info = "(ArmLeaveTote) "
            print self.info
            
 
            #self.Arm.pub_ikCmd('ptp', (0.25, 0.0 , 0.2), (-90, 0, 0) )
            #self.Arm.pub_ikCmd('ptp', (0.2, 0.0 , 0.25), (-90, 0, 0) )
            #self.Arm.pub_ikCmd('ptp', (0.25, 0.0 , 0.2), (-90, 0, 0) )
           
            
            
            pos = self.Arm.get_fb().group_pose.position

            if pos.x > 0.40:
                self.arm_leave_tote_safe()
                
                while self.Arm.busy:
                    rospy.sleep(0.1)
        


            self.arm_leave_tote()
            
            if self.now_stow_info.gripper_down:
                gripper_suction_down()
            else:
                gripper_suction_up()

            self.next_state = LM2Bin #ArmPutInBin
            self.state 		= WaitRobot
            return

        elif self.state == LM2Bin:       
            self.info = "(LM2Bin) LM LM2Bin, Go Bin -> " + self.now_stow_info.to_bin +' for item ->' + self.now_stow_info.item
            print self.info

            #show_stow_list(self.use_stow_list)
    
            self.Is_BaseShiftOK = False
            self.LM_2_Bin_Right_Arm(self.now_stow_info.to_bin)
            # self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', self.now_stow_info.to_bin ))
            # rospy.sleep(0.3)
            # self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', self.now_stow_info.to_bin ))
        
            

            self.next_state 	= ArmPutInBin #ArmLeaveTote   
            self.state 			= WaitRobot

            return

        elif self.state == ArmPutInBin:  
            self.info = "(ArmPutInBin) Arm ArmPutInBin "
            print self.info

            self.next_state = GripperOff
            self.state 		= WaitRobot

            #self.Arm.relative_move_nsa(a = 0.25) 
            #self.Arm.relative_move_nsa(a = 0.35) 
            self.Arm.relative_move_nsa(a = 0.2)
            return

        elif self.state == GripperOff:
            self.info = "(GripperOff) PutObj in GripperOff "
            print self.info
            
            if self.suck_num > 0:
                self.now_stow_info.success = True
                self.stow_success.append(self.now_stow_info)
            else:
                self.now_stow_info.success = False
                if self.use_stow_list == self.stow_fail:
                    self.stow_fail_2.append(self.now_stow_info)
                else:
                    self.stow_fail.append(self.now_stow_info)
            
            self.use_stow_list.remove(self.now_stow_info)

            gripper_vaccum_off()

            self.next_state = ArmLeaveBin
            self.state = WaitRobot
            return

        elif self.state == ArmLeaveBin:       
            self.info = "(ArmLeaveBin) ArmLeaveBin"
            print self.info
            
            self.Arm.relative_move_nsa(a = -0.2) 

            

            self.next_state = Recover2InitPos
            self.state 		= WaitRobot


            

            return

        elif self.state == Recover2InitPos:     
            self.info = "(Recover2InitPos) Arm Recover2InitPos "
            print self.info

            #self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
            #self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (-90, 0, 0) )
            self.arm_init_pose()

            self.next_state = FinishOne
            self.state 		= WaitRobot
            
            return

        # ===============================================================

        elif self.state == WaitRobot:
            # rospy.sleep(0.3)
            change_next_state = False

            #print 'self.Last_LMArrive == ' + str(self.Last_LMArrive) + ' and  self.Is_LMArrive ==' + str(self.Is_LMArrive) + ' and self.Is_LMBusy == ' + str(self.Is_LMBusy)


            #if self.Last_LMArrive == False and self.Is_LMArrive == True and self.Is_ArmBusy == False:
            if not self.Is_BaseShiftOK and self.LM.IsArrive and  self.Last_LMArrive == True and self.Is_LMArrive == True and self.Is_LMBusy == False and  self.Is_ArmBusy == False:
            #if not self.Is_BaseShiftOK and self.Last_LMArrive and self.Is_LMArrive and  not self.Is_LMBusy  and not self.Is_ArmBusy:
            #if not self.Is_BaseShiftOK and self.Last_LMArrive and self.Is_LMArrive and  not self.Is_LMBusy :
                self.Is_BaseShiftOK = True
                change_next_state = True
                print 'LM Postive trigger'
            elif self.Is_BaseShiftOK == True and self.Is_ArmBusy == False:
                change_next_state = True
                print '~Robot Ready~'

            # if not self.Is_BaseShiftOK and self.Is_LMArrive and not self.Is_LMBusy:
            #     self.Is_BaseShiftOK = True
            #     # self.state = self.next_state
            #     print('LM Positive trigger')
            # elif self.Is_BaseShiftOK and not self.Is_ArmBusy:
            #     # self.state = self.next_state
            #     change_next_state = True
            #     print('Robot Move Done')

            # if self.LM.is_free  and not self.Is_ArmBusy:
            #     change_next_state = True
            #     print '~Robot Ready~'


            #print 'self.Arm.is_ikfail = ' + str(self.Arm.is_ikfail)
            if self.Arm.is_ikfail:
                rospy.logwarn('In Strategy IK Fail')
                self.Arm.init()
                if self.state 	== ArmPutInBin or self.state == GripperOff or self.state == ArmLeaveBin or \
                    self.next_state == ArmPutInBin or self.next_state == GripperOff or self.next_state == ArmLeaveBin:
                    rospy.logwarn('In IK Fail, dangerous pose')
                    self.arm_leave_tote()
                    while self.self.Arm.busy:
                        rospy.sleep(0.1)


                change_next_state = True


            if change_next_state:
                
                if self.mode == Mode_KnownProcess:
                    print('self.mode = Mode_KnownProcess')
                    if self.check_vaccum_by_next_state(self.next_state):
                        self.state 			= self.next_state
                    else:
                        self.now_stow_info.success = False
                        
                        #self.stow_fail.append(self.now_stow_info)
                        if self.use_stow_list == self.stow_fail:
                            self.stow_fail_2.append(self.now_stow_info)
                        else:
                            self.stow_fail.append(self.now_stow_info)
                        
                        self.use_stow_list.remove(self.now_stow_info)

                        self.state = FinishOne
                else:

                    print('self.mode = Mode_UnknownProcess')

                    if self.check_vaccum_by_next_state(self.next_state):
                        self.state 			= self.next_state
                    else:
                        self.state 			= PhotoPose
                        
                    #self.state 			= self.next_state

            return

        elif self.state == FinishOne:
            self.mlog('FinishOne')
            # gripper_vaccum_off()      move to LM2Tote
            can_get_one = self.stow_get_one()

            if can_get_one :
                self.Is_BaseShiftOK = False
                self.state 			= PhotoPose 
                self.next_state 	= WaitTask

                #self.info = 'Finish' 
            else: 
                self.state 			= EndTask 
                
            
            self.print_all_list()
        

#------------------------Amnesty for Unknown Object--------------------------#
        elif self.state == LM2Amnesty_Up: 
            self.mlog('(LM2Amnesty_Up)')
            #self.LM_amnesty_up()
            self.Is_BaseShiftOK = False
            #self.LM.rel_move_LM('base', -30)
            #self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'amnesty'))
    
            self.LM_amnesty_up()

            self.state 			= WaitRobot
            self.next_state 	= LM2Amnesty_Down 
            #self.next_state 	= WaitTask
            return

        elif self.state == LM2Amnesty_Down: 
            self.mlog('(LM2Amnesty_Down)')
            self.Is_BaseShiftOK = False
            #self.LM.rel_move_LM('right', -35)
            #self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'amnesty'))
            self.LM_amnesty_down()

            self.state 			= WaitRobot
            self.next_state 	= AmnestyGripperOff 
            return

        elif self.state == AmnestyGripperOff: 
            self.mlog('(AmnestyGripperOff)')
            gripper_vaccum_off()

            
            
            self.state 			= WaitRobot
            self.next_state 	= Amnesty_End  #PhotoPose
            return


        elif self.state == Amnesty_End: 
            self.mlog('(Amnesty_End)')
            #self.LM_amnesty_up()

            self.LM.pub_LM_Cmd(LM_ID_Right, GetShift('Tote', 'z', 'tote'))
            self.arm_photo_pose()
            rospy.sleep(0.3)
            #self.LM_2_tote()
            self.LM.pub_LM_Cmd(LM_ID_Base, GetShift('Tote', 'x', 'tote'))
            
        

            self.unknown_change_2_known()
            

            self.state 			= FinishOne

            return


        elif self.state == EndTask:
            self.mlog('(EndTask)')
            self.dump_stow_list_2_item_location_file()

            self.print_all_list()
            self.task_finish()
            self.info = "Finish Stow Task"
            rospy.loginfo(self.info)	
            
            
        else:
            return

        
# Test------------------------------Test--------------------------------------------------
    def mlog(self, msg):
        print msg
    
    def print_stow_list_item(self, i_list, name):
        p = name + " : {"
        for stow_info in i_list:
            p = p + stow_info.item + ", "
            
        p = p + "}"

        print(p)

    def print_all_list(self):
        self.print_stow_list_item(self.stow_list, "stow_list")
        self.print_stow_list_item(self.stow_success, "stow_success")
        self.print_stow_list_item(self.stow_fail, "stow_fail")
        self.print_stow_list_item(self.stow_fail_2, "stow_fail_2")
        self.print_stow_list_item(self.use_stow_list, "use_stow_list")

    def test_obj_pose_done(self, status, result):
        self.obj_pose = result.object_pose
        if result.object_pose.linear.z == -1:
            rospy.logwarn('ROI Fail!! obj -> ' + self.now_stow_info.item)
            self.state = WaitTask
            return 
        else:
            self.obj_pose = result.object_pose
            print '(x, y , z) = ' + '(' + str(self.obj_pose.linear.x) + ', ' + str(self.obj_pose.linear.y) + ', ' + str(self.obj_pose.linear.z) + ')'
            
            p = self.obj_pose
            # print(str(self.obj_pose))
            rospy.loginfo("(x,y,z)= (" + str(p.linear.x) + ", " + str(p.linear.y)+ ", " + str(p.linear.z)  + ')' )
            rospy.loginfo("(roll,pitch,yaw)= (" 
                            + str(numpy.rad2deg(p.angular.x)) + ", " 
                            + str(numpy.rad2deg(p.angular.y)) + ", " 
                            + str(numpy.rad2deg(p.angular.z))    + ")"  
            )
        

    def test_obj_pose(self,want_item):
        rospy.loginfo("Request " + want_item)
        goal = obj_pose.msg.ObjectPoseGoal(want_item)

        self.obj_pose_client.send_goal(
                    goal,
                    feedback_cb = self.obj_pose_feedback_cb, 
                    done_cb=self.test_obj_pose_done )

    def test_run_with_obj_pose(self, cam_x, cam_y, cam_z):
        move_cam_x = cam_x
        move_cam_y = cam_y - cam2tool_y
        move_cam_z = cam_z - cam2tool_z

        rospy.loginfo("cam2tool_y = " + str(cam2tool_y) +", cam2tool_z=" + str(cam2tool_z))
        rospy.loginfo('move linear n(cam_y)='+str(move_cam_y) + ', s(cam_x)='+str(move_cam_x)  + ', a(cam_z)='+str(move_cam_z))

        # n = move_cam_y , s= -move_cam_x, a = move_cam_z
        self.Arm.relative_move_nsa(n= move_cam_y, s = move_cam_x, a = move_cam_z)


        # self.Arm.relative_control(a=move_cam_z)

    def test_read_item_location_in_arc_pack(self, f_name = "stow_test.json"):
        # read json in arc package
        rospack = rospkg.RosPack()
        item_location_path = rospack.get_path('arc') + "/stow_task/" +  f_name
        print('item_location_path = ' + item_location_path)
        f = open(item_location_path, 'r')
        read_data = f.read()

        # save_item_location
        self.save_item_location(read_data)

    def test_2_stow_fail(self):
        self.now_stow_info.success = False
        self.stow_fail.append(self.now_stow_info)
        self.use_stow_list.remove(self.now_stow_info)

        self.print_stow_list_item(self.use_stow_list, "stow_list")
        self.print_stow_list_item(self.stow_fail, "stow_fail")


    def test_dump_stow_success(self):
        self.test_read_item_location_in_arc_pack("stow_10.json")
        self.run()

        s = self.get_stow_info_by_name('robots_everywhere')
        s.to_bin = 'e'
        self.stow_success.append(s)
        self.use_stow_list.remove(s)

        s = self.get_stow_info_by_name('scotch_sponges')
        s.to_bin = 'b'
        self.stow_success.append(s)
        self.use_stow_list.remove(s)

        s = self.get_stow_info_by_name('robots_dvd')
        s.to_bin = 'a'
        self.stow_success.append(s)
        self.use_stow_list.remove(s)


        s = self.get_stow_info_by_name('speed_stick')
        s.to_bin = 'a'
        self.stow_success.append(s)
        self.use_stow_list.remove(s)

        self.dump_stow_list_2_item_location_file()

        self.print_all_list()

    def test_all_unknown_2_amnesty(self):
        self.mode = Mode_UnknownProcess
        self.state = LM2Tote

    def test_done_cb(self, state,result):
        print('result.object_pose = ' + str(result.object_pose))
        print('result.norm = ' + str(result.norm))
        print '-----------obj_pose_done_cb---[highest]---<< ' + result.object_name +' >>--------------'


    # def test_Distribution(self):
    #     mission_objects = ['avery_binder','burts_bees_baby_wipes','crayons','epsom_salts','fiskars_scissors',
    #                 'ice_cube_tray','tennis_ball_container','reynolds_wrap','hanes_socks','colgate_toothbrush_4pk',
    #                 ]

    #     output = Distribution('pick',mission_objects,0.5)

    #     print('===============output===============')
    #     print(output)

    def test_request_closest_sift(self):
        
        goal = obj_pose.msg.ObjectPoseGoal(
            object_name = "<Closest>",
            # object_list = ["laugh_out_loud_jokes",
            #                 "scotch_sponges",
            #                 "duct_tape",
            #                 "band_aid_tape",
            #                 "irish_spring_soap",
            #                 "crayons",
            #                 "expo_eraser",
            #                 "ice_cube_tray",
            #                 "robots_dvd"],
            object_list = ["tissue_box",
                             "duct_tape"],

            #          [ xmin, xmax, ymin, ymax, zmin, zmax]
            #limit_ary =[-0.15, 0.15, 0,  0.3, 0.3, 1.0]
            limit_ary = self.vision_limit_ary
        )

        self.obj_pose_client.send_goal(
                goal,
                done_cb=self.test_done_cb )
    
    def test_request_unknown_highest_item(self):
        fix_ary = [0.03, -0.03, 0.02, -0.02, -0.05,0]

        new_limit_ary = []
        for i in range(len(fix_ary)):
            print('i='+str(i))
            new_limit_ary.append(self.vision_limit_ary[i] +fix_ary[i])



        print 'new_limit_ary=' + str(new_limit_ary)
        print 'self.vision_limit_ary=' + str(self.vision_limit_ary)



        goal = obj_pose.msg.ObjectPoseGoal(
            object_name = "<Unknown_Closest>",
            #limit_ary =[-0.12, 0.12, -0.1,  0.4, 0.35, 0.6],
            limit_ary = new_limit_ary
        )
        self.obj_pose_client.send_goal(
                goal,
                feedback_cb = self.obj_pose_feedback_cb, 
                done_cb=self.test_done_cb )

    def test_request_highest(self):
        
        goal = obj_pose.msg.ObjectPoseGoal(
            object_name = "<Closest>",
            # object_list = ["laugh_out_loud_jokes",
            #                 "scotch_sponges",
            #                 "duct_tape",
            #                 "band_aid_tape",
            #                 "irish_spring_soap",
            #                 "crayons",
            #                 "expo_eraser",
            #                 "ice_cube_tray",
            #                 "robots_dvd"],
            object_list = ["tissue_box",
                             "duct_tape"],

            #          [ xmin, xmax, ymin, ymax, zmin, zmax]
            limit_ary =[-0.15, 0.15, 0,  0.3, 0.3, 1.0]
            
        )

        self.obj_pose_client.send_goal(
                goal,
                done_cb=self.test_done_cb )


    