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
			if self.stow.is_ready() :
				rospy.loginfo('Stow Task Running')
				self.run_task_type = TaskType_Stow
				self.stow.run()  
			else:
				rospy.logwarn('Pick Task Not Ready!!')
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
					info_json = self.pick.get_info()
				elif self.run_task_type == TaskType_Stow:
					self.stow.stow_core()
					info_json = self.stow.get_info()
					
				self.info_pub.publish(json.dumps(info_json))
			
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
		self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.22), (0, 0, 0) )

	def arm_bin_photo(self):
		self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.27), (0, 0, 0) )


	def safe_pose(self):
		self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (-90, 0, 0) )

if __name__ == '__main__':
	rospy.init_node('strategy')

	try:
		s = Strategy()
		#s.arm_go_init_pose()
		# s.start() 
		#s.arm_go_init_pose()
		s.start() 
		#s.test_go_bin_LM('j')
		#test_go_box('j')		
		# s.stow.LM_2_tote()			# -
		#s.stow.arm_photo_pose()		# -

		# s.safe_pose()
		#gripper_vaccum_off()
		#gripper_suction_up()
		#gripper_suctoin_down()
		#s.Arm.relative_control(a=.035)  #cam_z

		# s.stow.test_obj_pose('expoEraser')  #expoEraser

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
