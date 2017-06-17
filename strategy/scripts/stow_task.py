#!/usr/bin/env python

# pylint: disable = invalid-name
# pylint: disable = C0326, C0121, C0301
# pylint: disable = W0105, C0303, W0312

import math
import threading
import time

import actionlib
import roslaunch
import roslib;roslib.load_manifest('obj_pose')
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

# Define State
WaitTask  		= 1		# Wait Task
ParseJSON   	= 2		# Parse Json
Down2Pick   	= 3		# Move down to pick object in bin.
Init_Pos		= 4		# Make robot arm go to the initial pos
Go2Bin			= 5		# Make robot arm go to the specify bin
WaitRobot   	= 6		# wait for robot complete task
Up2LeaveBin 	= 7 	# Move up to leave bin (robot arm still in bin)
LeaveBin		= 8		# Make robot arm leave bin
FinishTask  	= 9		
LM_Test1  		= 10
PickObj 		= 11
PlaceObj		= 12
Move2PlaceObj1 	= 13
Move2PlaceObj2 	= 14
Recover2InitPos = 15

# Stow_Task
Shift2Tote		= 16
Go2Tote			= 17
Up2LeaveTote	= 18
Trans2StowOri	= 19
Shift2Stow		= 20
Go2Stow			= 21
StowObj			= 22

Shift2Del 		= 24
DelObj 			= 25
PhotoPose		= 26
VisionProcess	= 27
WaitVision		= 28

Rotate2Obj		= 29
Shift2Obj 		= 30
Arm_Down_2_Obj  = 31

class StowTask:
	""" description """
	def __init__(self, i_arm, i_LM):
		
		self.Arm = i_arm
		self.LM = i_LM
		self.obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)

		
		self.var_init()		
        
		rospy.loginfo('StowTask::init()')


	def var_init(self):
		# === Initialize State === 
		self.state 			= ParseJSON #Shift2Tote
		self.next_state 	= WaitTask
		self.Is_ArmBusy    	= False
		self.Is_LMBusy     	= False
		self.Last_LM_Busy  	= False
		self.Is_LMArrive   	= True
		self.Last_LMArrive 	= True
		self.Is_BaseShiftOK = False 
		self.Is_ArmMoveOK 	= False

		self.BinCnt 	   	= 0
		self.Box 		   	= 'a'
		self.Tote 		   	= 'a'
        
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
		

	def obj_pose_feedback_cb(self,fb):
		rospy.loginfo("In obj_pose_feedback_cb")
		rospy.loginfo("msg = " + fb.msg)
		rospy.loginfo("progress = " + str(fb.progress) + "% ")

        
	def obj_pose_done_cb(self, state, result):
		self.obj_pose = result.object_pose
		if result.object_pose.linear.z == -1:
			rospy.logwarn('ROI Fail!! obj -> ' + self.now_stow_info.item)
			self.state = WaitTask
			return 
		else:
			self.obj_pose = result.object_pose
			#self.state = Rotate2Obj
			self.state = Shift2Obj


	def run(self):
		self.stow_id = 0
		self.stow_get_one()
		self.state 	= Shift2Tote#Init_Pos

	def is_ready(self):
		if self.stow_list != None:
			return True
		else: 
			return False

	def stow_get_one(self):
		""" Get one pick task """
		""" Set: self.Bin, self.pick_id, self.Box """
		if self.stow_id >= len(self.stow_list):
			return False
		
		self.now_stow_info = self.stow_list[self.stow_id]
		
		self.stow_id = self.stow_id + 1
		return True

		
	def LM_2_tote(self):
		self.info = "(GoTote) Shift 2 tote "  
		print self.info

		# self.next_state 	= Init_Pos  
		# self.state 			= WaitRobot
		self.Is_BaseShiftOK = False
		self.LM.pub_LM_Cmd(2, GetShift('Tote', 'x', self.Tote ))
		rospy.sleep(0.3)
		self.LM.pub_LM_Cmd(1, GetShift('Tote', 'z', self.Tote ))

	def arm_photo_pose(self):
		self.info = "(Catch) Go PhotoPose "  
		print self.info

		self.next_state = VisionProcess
		self.state 		= WaitRobot
		self.Arm.pub_ikCmd('ptp', (0.40, 0.00 , 0.15), (-90, 0, 0) )

	def stow_core(self):
		self.update_status()

		if self.state == WaitTask:
			return
		if self.state == WaitVision:
			return
		elif self.state == Shift2Tote: 
			self.LM_2_tote()
			gripper_suction_up()

			self.state 			= WaitRobot
			self.next_state 	= PhotoPose #Init_Pos
			return

		elif self.state == Init_Pos:       # step 1
			self.info = "(GoTote) Arm Init_Pos "  
			print self.info

			#self.next_state = Go2Tote
			self.next_state = Shift2Tote
			self.state 		= WaitRobot
			#self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
			self.Arm.pub_ikCmd('ptp', (0.30, 0.0 , 0.22), (0, 0, 0)  )
			
			return

		elif self.state == PhotoPose:    
			self.arm_photo_pose()
			gripper_suction_up()
			return`

		elif self.state == VisionProcess:    
			self.info = "(Catch) Request  Vision Process "  
			print self.info

			goal = obj_pose.msg.ObjectPoseGoal(self.now_stow_info.item)

			self.obj_pose_client.send_goal(
					goal,
					feedback_cb = self.obj_pose_feedback_cb, 
					done_cb=self.obj_pose_done_cb )

			self.next_state = Shift2Obj
			self.state 		= WaitVision
			
			return
		
		elif self.state == Rotate2Obj:
			self.info = "(Catch) Arm Rotate2Obj "  
			print self.info


			pitch = numpy.rad2deg(p.angular.x ) 
			pitch = (pitch - 180) if pitch > 90  else pitch
			pitch = (pitch + 180) if pitch < -90  else pitch
			pitch = pitch *(-1)
			self.relative_control_rotate( pitch = pitch)

			self.next_state = Shift2Obj
			self.state 		= WaitRobot
			
			return

		elif self.state == Shift2Obj:
			self.info = "(Catch) Arm Shift2Obj "  
			print self.info

			l = self.obj_pose.linear
			#cam axi, tool need to move 
			move_cam_x = l.x
			move_cam_y = l.y - cam2tool_y
			move_cam_z = l.z - cam2tool_z
			
			rospy.loginfo('move linear n(cam_y)='+str(move_cam_y) + ', s(cam_x)='+str(move_cam_x)  + ', a(cam_z)='+str(move_cam_z))
			#self.relative_xyz_base(x = )
			
			rospy.loginfo('move linear base_x='+str(-move_cam_y) +
				', base_y='+str(move_cam_x)  +
				', base_z='+str(-move_cam_z))
			

			#self.relative_control(n = move_cam_y , s= -move_cam_x, a = move_cam_z)
			self.Arm.relative_xyz_base(x = -move_cam_y, y = move_cam_x,z = -move_cam_z)
		

			self.next_state = Arm_Down_2_Obj #PickObj
			self.state 		= WaitRobot
			
			return

		elif self.state == Arm_Down_2_Obj:    
			self.info = "(GoTote) Arm_Down_2_Obj "  
			print self.info

			self.next_state = PickObj
			self.state 		= WaitRobot
			
			self.Arm.relative_control(a=.035)  #cam_z
			
			return

		elif self.state == Go2Tote:    
			self.info = "(GoTote) Arm Go2Tote "  
			print self.info

			self.next_state = PickObj
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0), (-90, 0, 0) )
			
			
			return



		elif self.state == PickObj:	      # Enable Vacuum 
			self.info = "(GoTote) Arm Pick Obj " + self.now_stow_info.item
			print self.info
			
			self.next_state = Up2LeaveTote
			self.state = WaitRobot
			gripper_vaccum_on()
			
			return

		elif self.state == Up2LeaveTote:
			self.info = "(GoBin) Arm Up2LeaveTote "
			print self.info
			

			if self.BinCnt == 2: 				# Change 2 "IsRedundancy" in future after add image info
				self.next_state = Shift2Del
			else: 
				self.next_state = Trans2StowOri
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
			
			return

		elif self.state == Shift2Del:       
			self.next_state 	= DelObj  
			self.state 			= WaitRobot
			self.Tote 			= 'b'
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, GetShift('Tote', 'x', self.Tote ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, GetShift('Tote', 'z', self.Tote ))
			self.Tote 			= 'a'
			print 'Shift2Del'
			return

		elif self.state == Trans2StowOri: 
			self.info = "(GoBin) Arm Trans2StowOri "
			print self.info
			
			self.next_state = Go2Bin
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (0, 0, 0) )

			#-------Note----------#
			gripper_suction_down()

			return

		elif self.state == Go2Bin:       
			self.info = "(GoBin) LM Go2Bin "
			print self.info

			self.next_state 	= Go2Stow  
			self.state 			= WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', self.now_stow_info.to_bin ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', self.now_stow_info.to_bin ))
			
			
			return

		elif self.state == Go2Stow:  
			self.info = "(GoBin) Arm Go2Stow "
			print self.info

			self.next_state = StowObj
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0.0 , 0.2), (0, 0, 0) )

			return

		elif self.state == StowObj or self.state == DelObj:				# Disable Vacuum 
			
			if self.state == StowObj:
				self.next_state = LeaveBin
				self.info = "(GoBin) PutObj in StowObj "
			else:
				self.next_state = FinishTask
				self.info = "(GoBin) PutObj in DelObj "

			print self.info
			
			self.state = WaitRobot
			gripper_vaccum_off()
			
			return

		elif self.state == LeaveBin:       
			self.info = "(GoBin) LeaveBin"
			print self.info

			self.next_state = Recover2InitPos
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (0, 0, 0) )

			return

		elif self.state == Recover2InitPos:     
			self.info = "(GoBin) Arm Recover2InitPos "
			print self.info

			self.next_state = FinishTask
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )

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
			can_get_one = self.stow_get_one()

			if can_get_one :
				self.Is_BaseShiftOK = False
				self.state 			= Shift2Tote 
				self.next_state 	= WaitTask

				self.info = 'Stow one obj success' 
			else: 
				self.task_finish()
				self.info = "Finish Pick Task"
			
			print self.info	

		else:
			return



	def save_item_location(self,item_location_file):
		rospy.loginfo("[Stow] Save item_location")

		self.item_location_file = item_location_file
		self.stow_list = make_stow_list(item_location_file)		

	def get_info(self):
		info_json = {'info': self.info, 
				'item': self.now_stow_info.item, 
				'bin': self.now_stow_info.to_bin
				}
		
		return info_json

	def update_status(self):
		""" update ARM & LM status  """
		self.Last_LM_Busy 	= self.Is_LMBusy
		self.Last_LMArrive 	= self.Is_LMArrive
		self.Is_ArmBusy 	= self.Arm.busy
		self.Is_LMBusy  	= self.LM.IsBusy
		self.Is_LMArrive	= self.LM.IsArrive
