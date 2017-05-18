#!/usr/bin/env python

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

#import strategy

#from strategy.srv import *
#from arc_ui.srv import *





WaitTask  				= 1			# Wait Task
ParseJSON				= 2			# Parse Json


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



	def var_init(self):
		self.stop_robot = False

	def shutdown(self):
		self.stop_robot = True
		rospy.loginfo("Strategy Exit & Stop Robot")


	def task_cb(self,req):
		self.state = ParseJSON
		self.task_name = req.task_name
		json = req.task_json
		
		rospy.loginfo("task_name = " + self.task_name)
	

	def core(self):

		if self.stop_robot == True:
			return

		if self.state == WaitTask:
			return
		elif self.state == ParseJSON:	
			# in task_cb()
			return
		
		elif self.state == PickOne:	
			# in task_cb()
			self.next_state = OutBin
			self.state = WaitRobot

			return
		elif self.state == GoBin:	
			# in task_cb()
			#control.......
			self.next_state = PickOne
			self.state = WaitRobot

			return
		elif self.state == WaitRobot:
			if self._arm_busy == False and self._linear_motion_busy == False:
				self.state = self.next_state


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
