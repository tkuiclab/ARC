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
from task_parser import *
from config import *

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
LeaveTote		= 23
Shift2Del 		= 24
DelObj 			= 25


class Pick:
	item = ""
	from_bin = ""
	to_box = ""

class StowTask:
	""" description """
	def __init__(self, i_arm, i_LM):
		
		self.Arm = i_arm
		self.LM = i_LM

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

		self.Bin 		   	= 'a'
		self.BinCnt 	   	= 0
		self.Box 		   	= 'a'
		self.Tote 		   	= 'a'
        
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

		
	def stow_core(self):
		self.update_status()

		if self.state == WaitTask:
			return
		elif self.state == ParseJSON:	
			return
		elif self.state == Shift2Tote:       
			self.next_state 	= Init_Pos  
			self.state 			= WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, self.GetShift('Tote', 'x', self.Tote ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, self.GetShift('Tote', 'z', self.Tote ))
			print 'Shift 2 tote'
			return

		elif self.state == Init_Pos:       # step 1
			self.next_state = Go2Tote
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
			print 'Init_Pos'
			return

		elif self.state == Go2Tote:       
			self.next_state = PickObj
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0), (-90, 0, 0) )
			print 'Go2Tote'
			return

		elif self.state == PickObj:	      # Enable Vacuum 
			self.next_state = Up2LeaveTote
			self.state = WaitRobot
			self.LM.Vaccum_Test(True)
			rospy.sleep(1)
			print 'PiclObj'
			print '========== Enable Vacuum =========='
			return

		elif self.state == Up2LeaveTote:      
			if self.BinCnt == 2: 				# Change 2 "IsRedundancy" in future after add image info
				self.next_state = Shift2Del
			else: 
				self.next_state = Trans2StowOri
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
			print 'Up2LeaveTote'
			return

		elif self.state == Shift2Del:       
			self.next_state 	= DelObj  
			self.state 			= WaitRobot
			self.Tote 			= 'b'
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, self.GetShift('Tote', 'x', self.Tote ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, self.GetShift('Tote', 'z', self.Tote ))
			self.Tote 			= 'a'
			print 'Shift2Del'
			return

		elif self.state == Trans2StowOri:       
			self.next_state = Shift2Stow
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (0, 0, 0) )
			print 'Trans2StowOri'
			return

		elif self.state == Shift2Stow:       
			self.next_state 	= Go2Stow  
			self.state 			= WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, self.GetShift('Bin', 'x', self.Bin ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, self.GetShift('Bin', 'z', self.Bin ))
			print 'Shift2Stow'
			return

		elif self.state == Go2Stow:       
			self.next_state = StowObj
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0.0 , 0.2), (0, 0, 0) )
			print 'Go2Stow'
			return

		elif self.state == StowObj or self.state == DelObj:				# Disable Vacuum 
			if self.state == StowObj:
				self.next_state = LeaveTote
			else:
				self.next_state = FinishTask
			self.state = WaitRobot
			self.LM.Vaccum_Test(False)
			print '========== Disable Vacuum =========='
			print 'StowObj or DelObj'
			return

		elif self.state == LeaveTote:       
			self.next_state = Recover2InitPos
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (0, 0, 0) )
			print 'LeaveTote'
			return

		elif self.state == Recover2InitPos:       
			self.next_state = FinishTask
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
			print 'Recover2InitPos'
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
			""" Continue exe next bin """
			print 'self.BinCnt'
			self.BinCnt = self.BinCnt + 1
			if self.BinCnt >= 3:
				""" Recover to initial status (Constructor) """
				self.state			= WaitTask
				self.next_state 	= WaitTask
				self.Is_ArmBusy 	= False
				self.Is_LMBusy		= False
				self.Last_LM_Busy 	= False
				self.Is_LMArrive   	= True
				self.Last_LMArrive 	= True
				self.Is_BaseShiftOK = False
				self.BinCnt 		= 0
				print 'Finish Stow Task'
				return
			else:
				self.Is_BaseShiftOK = False
				self.Bin 			= BinId[self.BinCnt]
				self.state 			= Shift2Tote 
				self.next_state 	= WaitTask

				print 'Finish one obj in stow task' 
				print self.BinCnt
			# ===============================================
				# elif self.state == FinishTask:
				# 	""" Continue exe next bin """
				# 	print 'self.BinCnt'
				# 	if self.BinCnt > 12:
				# 		""" Recover to initial status (Constructor) """
				# 		self.state			= WaitTask
				# 		self.next_state 	= WaitTask
				# 		self.Is_ArmBusy 	= False
				# 		self.Is_LMBusy		= False
				# 		self.Last_LM_Busy 	= False
				# 		self.Is_LMArrive   	= True
				# 		self.Last_LMArrive 	= True
				# 		self.Is_BaseShiftOK = False
				# 		return
				# 	else:
				# 		self.Is_BaseShiftOK = False
				# 		self.BinCnt = self.BinCnt + 1
				# 		self.Bin 	= BinId[self.BinCnt]
				# 		self.state 	= LM_Test1 
				# 		self.next_state 	= WaitTask
			# ===============================================
		else:
			return
		

	def update_status(self):
		""" update ARM & LM status  """
		self.Last_LM_Busy 	= self.Is_LMBusy
		self.Last_LMArrive 	= self.Is_LMArrive
		self.Is_ArmBusy 	= self.Arm.busy
		self.Is_LMBusy  	= self.LM.IsBusy
		self.Is_LMArrive	= self.LM.IsArrive


	def GetShift(self, Target_Type, LM_Dir, Target):
		""" Get the motion pulse of linear motor """
		# Bin
		if Target_Type == 'Bin':
			if Target in BinId:
				if LM_Dir == 'x':
					return BinShift_X[BinId.index(Target)]
				elif LM_Dir=='z':
					return BinShift_Z[BinId.index(Target)]
				else:
					print 'Error input Bin dir'
					return 0
			else:
				print 'Error input character'

		# Tote
		elif  Target_Type == 'Tote':
			if Target in ToteId:
				if LM_Dir == 'x':
					return ToteShift_X[ToteId.index(Target)]
				elif LM_Dir=='z':
					return ToteShift_Z[ToteId.index(Target)]
				else:
					print 'Error input Tote dir'
					return 0
			else:
				print 'Error input character'

		# Box
		elif  Target_Type == 'Box':
			if Target in BoxId:
				if LM_Dir == 'x':
					return BoxShift_X[BoxId.index(Target)]
				elif LM_Dir=='z':
					return BoxShift_Z[BoxId.index(Target)]
				else:
					print 'Error input dir'
					return 0
			else:
				print 'Error input Box character'
		
		else:
			print 'Error input Target Type'
