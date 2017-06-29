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

import obj_pose.msg
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String
from strategy.srv import *

import arm_task_rel
import LM_Control
from task_parser import *
from config import *
from gripper import *

# Define State jmp
WaitTask  		= 1		# Wait Task
ParseJSON   	= 2		# Parse Json
Down2Pick   	= 3		# Move down to pick object in bin.
Init_Pos		= 4		# Make robot arm go to the initial pos()
Go2Bin			= 5		# Make robot arm go to the specify bin
WaitRobot   	= 6		# wait for robot complete task
Up2LeaveBin 	= 7 	# Move up to leave bin (robot arm still in bin)
LeaveBin		= 8		# Make robot arm leave bin
FinishTask  	= 9		
Shift2Bin  		= 10
PickObj 		= 11
PlaceObj		= 12
Move2PlaceObj1 	= 13
Move2PlaceObj2 	= 14
Recover2InitPos = 15

PhotoPose		  = 26
VisionProcess	  = 27
Rotate2PickPosOri = 28
WaitVision 		  = 29

# Stow_Task
Shift2Tote		= 16
Go2Tote			= 17
Up2LeaveTote	= 18
Trans2StowOri	= 19
Shift2Stow		= 20
Go2Stow			= 21
StowObj			= 22
LeaveTote		= 23
Shift2Del 		= 24
DelObj 			= 25


class Pick:
	item = ""
	from_bin = ""
	to_box = ""

class PickTask:
	""" description """
	def __init__(self, i_arm, i_LM):

		self.Arm = i_arm
		self.LM = i_LM
		self.obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)

		self.var_init()		
        
		rospy.loginfo('PickTask::init()' )

	
	def var_init(self):
		self.pick_list = None
		# === Initialize State === 
		self.state 			= WaitTask
		self.next_state 	= WaitTask
		self.Is_ArmBusy    	= False
		self.Is_LMBusy     	= False
		self.Last_LM_Busy  	= False
		self.Is_LMArrive   	= True
		self.Last_LMArrive 	= True
		self.Is_BaseShiftOK = False 
		self.Is_ArmMoveOK 	= False

		self.Bin 		   	= 'e'
		self.BinCnt 	   	= 0


		self.Box 		   	= 'a'
		self.Tote 		   	= 'a'

		self.item_location = None
		self.order = None
        
	def task_finish(self):
		self.state			= WaitTask
		self.next_state 	= WaitTask
		self.Is_ArmBusy 	= False
		self.Is_LMBusy		= False
		self.Last_LM_Busy 	= False
		self.Is_LMArrive   	= True
		self.Last_LMArrive 	= True
		self.Is_BaseShiftOK = False
		print 'Finish Task'

	def save_item_location(self,item_location):
		rospy.loginfo("[Pick] Save item_location")

		self.item_location = item_location
		self.parse_json_2_pick_list()
		

	def save_order(self,order):
		rospy.loginfo("[Pick] Save order")
		self.order = order
		self.parse_json_2_pick_list()

	def parse_json_2_pick_list(self):
		if self.item_location==None or self.order==None:
			return
		rospy.loginfo("[Pick] Parse JSON of item_location and order to pick_list")
		self.pick_list = make_pick_list(self.item_location, self.order)
	
	def pick_get_one(self):
		""" Get one pick task """
		""" Set: self.Bin, self.pick_id, self.Box """
		if self.pick_id >= len(self.pick_list):
			return False
		
		
		self.now_pick = self.pick_list[self.pick_id]
		self.Bin = self.now_pick.from_bin.lower()
		self.Item = self.now_pick.item

		if self.now_pick.to_box == 'A1':
			self.Box = 'a'
		elif self.now_pick.to_box == '1A5':
			self.Box = 'b'
		elif self.now_pick.to_box == '1B2':
			self.Box = 'c'
		else:
			rospy.logerr('Error pick.to_box='+self.now_pick.to_box)
 
		self.pick_id = self.pick_id + 1
		
		return True

		
	def run(self):
		self.pick_id = 0
		self.pick_get_one()
		self.state 	= Shift2Bin

	def is_ready(self):
		if self.pick_list != None:
			return True
		else: 
			return False

	def obj_pose_done_cb(self, state, result):
		self.obj_pose = result.object_pose
		if result.object_pose.linear.z == -1:
			rospy.logwarn('ROI Fail!! obj -> ' + self.now_pick.item)
			self.state = WaitTask
			return 
		else:
			self.obj_pose = result.object_pose
			print 'obj_pose' + str(obj_pose)
			self.state 	  = Rotate2PickPosOri

	def obj_pose_feedback_cb(self,fb):
		rospy.loginfo("In obj_pose_feedback_cb")
		rospy.loginfo("msg = " + fb.msg)
		rospy.loginfo("progress = " + str(fb.progress) + "% ")

	def pick_core(self):
		self.update_status()

		if self.state == WaitTask:			
			return
		if self.state == WaitVision:
			return

		# elif self.state == ParseJSON:
		# 	self.info = "ParseJSON"

		# 	self.pick_list = read_config_pick_task()
		# 	# self.pick_id = 0
		# 	# self.pick_get_one()

		# 	#self.state 			= Shift2Bin
			
		# 	return
		
		elif self.state == Shift2Bin:      
			self.info = "(GoBin) LM Move To Bin " + self.Bin 
			print self.info

			self.next_state = Init_Pos   
			self.state 		= WaitRobot

			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', self.Bin ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', self.Bin ))
			return

		elif self.state == Init_Pos:     
			self.info = "(GoBin) Arm To Init_Pos" 
			print self.info
			self.next_state = PhotoPose
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (0, 0, 0) )
			return

		elif self.state == PhotoPose:       
			self.info = "(GoBin) Arm To PhotoPose" 
			print self.info
			self.next_state = VisionProcess
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.4), (-15, 0, 0) )
			return

		elif self.state == VisionProcess:    
			self.info = "(Catch) Request  Vision Process "  
			print self.info

			goal = obj_pose.msg.ObjectPoseGoal(self.now_pick.item)

			self.obj_pose_client.send_goal(
					goal,
					feedback_cb = self.obj_pose_feedback_cb, 
					done_cb     = self.obj_pose_done_cb )

			self.next_state = Rotate2PickPosOri
			self.state 		= WaitVision
			
			return

		elif self.state == Rotate2PickPosOri:  # read img info, trans to catch ori and move xyz relatively
			self.info = "(Catch) Arm Rotate 2 Pick Pos and Ori "
			print self.info

			self.next_state = Go2Bin
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (0, 0, 0) )  # tmp
			return

		elif self.state == Go2Bin:				# using nsa motion to pick obj.
			self.info = "(Catch) Arm Put in Bin " + self.Bin 
			print self.info

			self.next_state = PickObj
			self.state 		= WaitRobot
			self.Arm.relative_control(a=0.05)  
			return
		
		elif self.state == Down2Pick:	
			self.info = "(Catch) Arm Down to Catch "
			print self.info

			self.next_state = PickObj
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0, 0.145), (0, 0, 0) )

			return

		elif self.state == PickObj:	      # Enable Vacuum 
			self.info = "(Catch) Vacuum Enable "
			print self.info

			self.next_state = Up2LeaveBin
			self.state = WaitRobot
			gripper_vaccum_on()
			rospy.sleep(1)
			
			return

		elif self.state == Up2LeaveBin:
			self.info = "(Catch) Arm Up"
			print self.info

			self.next_state = LeaveBin
			self.state = WaitRobot
			self.Arm.relative_xyz_base(0, 0, 0.05)
			#print 'U2L'
			return

		elif self.state == LeaveBin: 
			self.info = "(Catch) Arm Leave Bin  "
			print self.info
			self.next_state = Move2PlaceObj1
			self.state = WaitRobot
			self.Arm.relative_xyz_base(-0.05, 0, 0)
			
			return

		elif self.state == Move2PlaceObj1:			# shift LM 2 go to tote
			self.info = "(GoBox) LM Go Box " +self.Box
			print self.info
			self.next_state 	= PlaceObj
			self.state 			= WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, GetShift('Box', 'x', self.Box ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, GetShift('Box', 'z', self.Box ))
			#rospy.sleep(1)
			
			return

		elif self.state == Move2PlaceObj2: 		# new added 1
			self.info = "(GoBox) Arm Go Box " +self.Box
			print self.info
			self.next_state = PlaceObj
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.22), (-90, 0, 0) )
			
			return

		elif self.state == PlaceObj:		# Disable Vacuum 
			# self.Is_BaseShiftOK = False
			self.info = "(GoBox) Vaccum Disable - [Success]"
			print self.info
			self.next_state = FinishTask
			self.state = WaitRobot
			gripper_vaccum_off()

			return

		elif self.state == Recover2InitPos: # new added 2
			self.info = "Arm Recover2InitPos"
			print self.info
			self.next_state = FinishTask
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.22), (0, 0, 0) )
			
			return

		# ===============================================================

		elif self.state == WaitRobot:
			# rospy.sleep(0.3)
			if self.Last_LMArrive == False and self.Is_LMArrive == True :
				self.Is_BaseShiftOK = True
				self.state 			= self.next_state
				print 'LM Postive trigger'

			elif self.Is_BaseShiftOK == True and self.Is_ArmBusy == False:
				self.state 			= self.next_state
				print 'BaseShiftOK'

			return

		elif self.state == FinishTask:
			can_get_one = self.pick_get_one()
			self.info = "Finish Pick One Object"
		
			if can_get_one :
				self.Is_BaseShiftOK = False 
				self.state 	= Shift2Bin 
				self.next_state 	= WaitTask
			else: 
				self.task_finish()
				self.info = "Finish Pick Task"
			print self.info	

			return
		else:
			return

	def get_info(self):
		info_json = {'info': self.info, 
				'item': self.Item, 
				'bin': self.Bin,
				'box': self.now_pick.to_box
				}
		
		return info_json

	def update_status(self):
		""" update ARM & LM status  """
		self.Last_LM_Busy 	= self.Is_LMBusy
		self.Last_LMArrive 	= self.Is_LMArrive
		self.Is_ArmBusy 	= self.Arm.busy
		self.Is_LMBusy  	= self.LM.IsBusy
		self.Is_LMArrive	= self.LM.IsArrive

	