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
WaitArm  		= 11
PickObj 		= 12
PlaceObj		= 13
Move2PlaceObj1 	= 14
Move2PlaceObj2 	= 15
Recover2InitPos = 16

Box_A1 = 'A1'
Box_1AD = '1AD'
Box_1A5 = '1A5'
Box_1B2 = '1B2'
Box_K3 = 'K3'

class Pick:
	item = ""
	from_bin = ""
	to_box = ""

class Strategy(threading.Thread):
	""" description """
	def __init__(self):
		threading.Thread.__init__(self)
		self.var_init()
		#ros shutdown
		rospy.on_shutdown(self.shutdown)
		rospy.Service('/task', Task, self.task_cb)
		rospy.loginfo("Strategy Ready!!")

		self.TargetId 	   	= [   'a',   'b',   'c',   'd',   'e',   'f',   'g',   'h',   'i',   'j',   'k',   'l' , 'z']
		self.TargetShift_X 	= [ 60000, 35000,     0, 60000, 35000,     0, 60000, 35000,     0, 60000, 35000,     0 , 59000]
		self.TargetShift_Z 	= [     0,     0,     0, 25000, 25000, 25000, 50000, 50000, 50000, 75000, 75000, 75000 , 80000]
		self.Bin 		   	= 'a'
		self.BinCnt 	   	= 0

		self.state 			= WaitTask
		self.next_state 	= WaitTask
		self.Arm 			= arm_task_rel.ArmTask()
		self.LM  			= LM_Control.CLM_Control()

		self.Is_ArmBusy    	= False
		self.Is_LMBusy     	= False
		self.Last_LM_Busy  	= False
		self.Is_LMArrive   	= True
		self.Last_LMArrive 	= True
		self.Is_BaseShiftOK = False 
		self.Is_ArmMoveOK 	= False


	def var_init(self):
		""" description """
		self.stop_robot = False

	def shutdown(self):
		""" description """
		self.stop_robot = True
		rospy.loginfo("Strategy Exit & Stop Robot")

	def task_cb(self,req):
		""" description """
		self.state = ParseJSON
		self.task_name = req.task_name
		#json = req.task_json
		rospy.loginfo("task_name = " + self.task_name)
		self.state = LM_Test1     # Init_Pos or LM_Test1

	def core(self):
		""" description """
		self.Last_LM_Busy 	= self.Is_LMBusy
		self.Last_LMArrive 	= self.Is_LMArrive
		self.Is_ArmBusy 	= self.Arm.busy
		self.Is_LMBusy  	= self.LM.IsBusy
		self.Is_LMArrive	= self.LM.IsArrive
		# print self.Is_LMArrive
		
		if self.stop_robot == True:
			return

		if self.state == WaitTask:
			return

		elif self.state == ParseJSON:	
			return
		
		elif self.state == LM_Test1:       # LM_test1
			self.next_state = Init_Pos   # note!!!!!!
			self.state 		= WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, self.GetShift('x', self.Bin ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, self.GetShift('z', self.Bin ))
			print '1'
			return

		elif self.state == Init_Pos:       # step 1
			self.next_state = Go2Bin
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.22), (0, 0, 0) )
			print 'Init_Pos'
			return

		elif self.state == Go2Bin:
			self.next_state = Down2Pick
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0, 0.22), (0, 0, 0) )
			print 'Go2Bin'
			return
		
		elif self.state == Down2Pick:	
			self.next_state = PickObj
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0, 0.145), (0, 0, 0) )
			print 'D2P'
			return

		elif self.state == PickObj:	      # Enable Vacuum 
			self.next_state = Up2LeaveBin
			self.state = WaitRobot
			self.LM.Vaccum_Test(True)
			rospy.sleep(1)
			print 'PiclObj'
			print '========== Enable Vacuum =========='
			return

		elif self.state == Up2LeaveBin:
			self.next_state = LeaveBin
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.6, 0, 0.22), (0, 0, 0) )
			print 'U2L'
			return

		elif self.state == LeaveBin: 
			self.next_state = Move2PlaceObj1
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.22), (0, 0, 0) )
			print 'LeaveBin'
			return

		elif self.state == Move2PlaceObj1:
			self.next_state = Move2PlaceObj2
			self.state = WaitRobot
			self.Is_BaseShiftOK = False
			self.LM.pub_LM_Cmd(2, self.GetShift('x', 'z' ))
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, self.GetShift('z', 'z' ))
			rospy.sleep(1)
			print 'Move2PlaceObj1'
			return

		elif self.state == Move2PlaceObj2: 		# new added 1
			self.next_state = PlaceObj
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.22), (-90, 0, 0) )
			print 'LeaveBin'
			return

		elif self.state == PlaceObj:		# Disable Vacuum 
			# self.Is_BaseShiftOK = False
			self.next_state = Recover2InitPos
			self.state = WaitRobot
			self.LM.Vaccum_Test(False)
			print '========== Disable Vacuum =========='
			print 'PlaceObj'
			return

		elif self.state == Recover2InitPos: # new added 2
			self.next_state = FinishTask
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.2), (0, 0, 0) )
			print 'LeaveBin'
			return

		# ===============================================================
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
			if self.BinCnt >= 12:
				""" Recover to initial status (Constructor) """
				self.state			= WaitTask
				self.next_state 	= WaitTask
				self.Is_ArmBusy 	= False
				self.Is_LMBusy		= False
				self.Last_LM_Busy 	= False
				self.Is_LMArrive   	= True
				self.Last_LMArrive 	= True
				self.Is_BaseShiftOK = False
				return
			else:
				self.Is_BaseShiftOK = False
				self.BinCnt = self.BinCnt + 1
				self.Bin 	= self.TargetId[self.BinCnt]
				self.state 	= LM_Test1 
				self.next_state 	= WaitTask
				print 'next'
				print self.BinCnt

		else:
			return

	def run(self):
		rate = rospy.Rate(30)  # 30hz
		while not rospy.is_shutdown():
			self.core()
			rate.sleep()

	def SetBin(self, tmpBin):
		""" description """
		self.Bin = tmpBin

	def GetShift(self, LM_Dir, Bin):
		""" Get the motion pulse of linear motor """
		if Bin in self.TargetId:
			if LM_Dir == 'x':
				return self.TargetShift_X[self.TargetId.index(Bin)]
			elif LM_Dir=='z':
				return self.TargetShift_Z[self.TargetId.index(Bin)]
			else:
				print 'Error input dir'
				return 0
		else:
			print 'Error input character'

if __name__ == '__main__':
	rospy.init_node('strategy')

	try:
		s = Strategy()
		# s.run_bin()
		# s.pick_ary()
		s.start()

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
