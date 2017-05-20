#!/usr/bin/env python

# pylint: disable = invalid-name
# pylint: disable = C0326, C0121
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
WaitTask  	= 1			# Wait Task
ParseJSON	= 2			# Parse Json
Down2Pick   = 3
Init_Pos	= 4
Go2Bin		= 5
WaitRobot   = 6
Up2LeaveBin = 7
LeaveBin	= 8
FinishTask  = 9
LM_Test1  = 10
LM_Test2 = 11
LM_Test3 = 12

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
	def __init__(self):
		threading.Thread.__init__(self)
		self.var_init()
		#ros shutdown
		rospy.on_shutdown(self.shutdown)
		rospy.Service('/task', Task, self.task_cb)
		rospy.loginfo("Strategy Ready!!")

		self.state = WaitTask
		self.next_state = WaitTask
		self.Arm = arm_task_rel.ArmTask()
		self.LM  = LM_Control.CLM_Control()
		self.Is_ArmBusy = False

		self.Is_LMBusy    = False
		self.Last_LM_Busy = False

		self.Is_LMArrive   = True
		self.Last_LMArrive = True

	def var_init(self):
		self.stop_robot = False

	def shutdown(self):
		self.stop_robot = True
		rospy.loginfo("Strategy Exit & Stop Robot")

	def task_cb(self,req):
		self.state = ParseJSON
		self.task_name = req.task_name
		#json = req.task_json
		rospy.loginfo("task_name = " + self.task_name)
		self.state = LM_Test1

	def core(self):
		""" description """
		self.Is_ArmBusy = self.Arm.busy

		self.Last_LM_Busy = self.Is_LMBusy
		self.Is_LMBusy  = self.LM.IsBusy

		self.Last_LMArrive = self.Is_LMArrive
		self.Is_LMArrive= self.LM.IsArrive
		
		# print self.LM.IsBusy
		
		if self.stop_robot == True:
			return

		if self.state == WaitTask:
			return

		elif self.state == ParseJSON:	
			return
		
		elif self.state == LM_Test1:       # LM_test1
			self.next_state = LM_Test2
			self.state 		= WaitRobot
			self.LM.pub_LM_Cmd(2, 10000)
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, 10000)
			print '1'
			return

		elif self.state == LM_Test2:       # LM_test2
			self.next_state = Init_Pos   # Init_Pos or LM_Test3
			self.state 		= WaitRobot
			self.LM.pub_LM_Cmd(2, 0)
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, 0)
			print '2'
			return

		elif self.state == LM_Test3:       # LM_test3
			self.next_state = FinishTask
			self.state 		= WaitRobot
			self.LM.pub_LM_Cmd(2, 10010)
			rospy.sleep(0.3)
			self.LM.pub_LM_Cmd(1, 10010)
			print '3'
			return

		elif self.state == Init_Pos:       # step 1
			self.next_state = Go2Bin
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.4, 0.1, 0.2), (-90, 0, 0) )
			print 'Init_Pos'
			return

		elif self.state == Go2Bin:
			self.next_state = Down2Pick
			self.state 		= WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.5, 0, 0.2), (-90, 0, 0) )
			print 'Go2Bin'
			return
		
		elif self.state == Down2Pick:	
			self.next_state = Up2LeaveBin
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.5, 0, 0.15), (-90, 0, 0) )
			print 'D2P'
			return

		elif self.state == Up2LeaveBin:
			self.next_state = LeaveBin
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.5, 0, 0.2), (-90, 0, 0) )
			print 'U2L'
			return

		elif self.state == LeaveBin:
			self.next_state = FinishTask
			self.state = WaitRobot
			self.Arm.pub_ikCmd('ptp', (0.4, 0, 0.2), (-90, 0, 0) )
			return

		elif self.state == WaitRobot:
			# print 'waiting'
			if self.Is_ArmBusy == False:
				self.state = self.next_state

			# if self.Last_LM_Busy == True and self.Is_LMBusy == False:
			if self.Last_LMArrive == False and self.Is_LMArrive == True:
				self.state = self.next_state

		elif self.state == FinishTask:

			""" Recover to initial status (Constructor) """
			self.state = WaitTask
			self.next_state = WaitTask
			self.Arm = arm_task_rel.ArmTask()
			self.LM  = LM_Control.CLM_Control()
			self.Is_ArmBusy = False

			self.Is_LMBusy    = False
			self.Last_LM_Busy = False

			self.Is_LMArrive   = True
			self.Last_LMArrive = True

			""" Continue exe next bin """

		else:
			return


	def run(self):
		rate = rospy.Rate(30)  # 30hz
		while not rospy.is_shutdown():
			self.core()
			rate.sleep()

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
