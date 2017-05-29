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
from pick_task import PickTask
from stow_task import StowTask


TaskType_Pick = 1
TaskType_Stow = 2

class Strategy(threading.Thread):
	""" description """
	def __init__(self):
		threading.Thread.__init__(self)
		rospy.on_shutdown(self.shutdown)
		rospy.Service('/task', Task, self.task_cb)
		
		# === Initialize All Var === 
		self.Arm 			= arm_task_rel.ArmTask()
		self.LM  			= LM_Control.CLM_Control()

		self.pick = PickTask(self.Arm, self.LM)		
		self.stow = StowTask(self.Arm, self.LM)

		self.stop_robot = False

		self.run_task_type = None

		rospy.sleep(0.3)
		rospy.loginfo("Strategy Ready!!")
		

	def shutdown(self):
		""" description """
		self.stop_robot = True
		rospy.loginfo("Strategy Exit & Stop Robot")

	def task_cb(self,req):
		""" description """
		task_name = req.task_name
		#json = req.task_json
		rospy.loginfo("task_name = " + task_name)
		if task_name.lower() == 'stow':
			self.run_task_type = TaskType_Stow
		elif task_name.lower() == 'pick':
			self.run_task_type = TaskType_Pick    
		else:
			print 'Error Task Name (Please input pick or stow)'
		
		r = TaskResponse()
		r.success = True
		r.msg = task_name.upper() + " Task Ready!"
		return r

	def run(self):
		rate = rospy.Rate(30)  # 30hz
		while not rospy.is_shutdown():
			#self.core()
			if self.stop_robot == True:
				return
			
			if self.run_task_type == TaskType_Pick:
				self.pick.pick_core()
			elif self.run_task_type == TaskType_Stow:
				self.stow.stow_core()
			
			rate.sleep()


	def test_go_bin_LM(self, bin):
		rospy.sleep(0.5)
		print 'test_go_bin_LM bin -> ' +bin  
		self.LM.pub_LM_Cmd(2, self.GetShift('Bin', 'x', bin ))
		rospy.sleep(0.3)
		self.LM.pub_LM_Cmd(1, self.GetShift('Bin', 'z', bin ))
			

	def test_go_box(self, box):
		rospy.sleep(0.5)
		print 'test_go_box_LM box -> ' + box  
		self.LM.pub_LM_Cmd(2, self.GetShift('Box', 'x', box ))
		rospy.sleep(0.3)
		self.LM.pub_LM_Cmd(1, self.GetShift('Box', 'z', box ))
		rospy.sleep(0.5)		

if __name__ == '__main__':
	rospy.init_node('strategy')

	try:
		s = Strategy()
		s.start() 
		
		#s.test_go_bin_LM('j')
		#s.test_go_box('a')		

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
