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
				rospy.logwarn('Stow Task Not Ready!!')
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
		self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.22), (0, 0, 0))

	def arm_bin_photo(self):
		self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.34), (0, 0, 0))


	def safe_pose(self):
		self.Arm.pub_ikCmd('ptp', (0.3, 0.0 , 0.3), (-180, 0, 0))

	def test_relative_move_nsa(self, dis = 0):
		# =======================================================================
		""" relative move nsa with a specify dis (safe when euler mode is 'nsa' mode) """
		# n = move_cam_y , s= -move_cam_x, a = move_cam_z
		# =======================================================================
		self.Arm.relative_move_nsa(n = -dis)
		self.Arm.relative_move_nsa(n =  dis)
		self.Arm.relative_move_nsa(s = -dis)
		self.Arm.relative_move_nsa(s =  dis)
		self.Arm.relative_move_nsa(a =  dis)
		self.Arm.relative_move_nsa(a = -dis)

	def test_relative_rot_nsa(self, rot = 0):
		# =======================================================================
		""" relative rotate pitch roll and yaw with a specifydegree """
		# (safe when euler mode is 'nsa' mode) 
		# when yaw(n) is not equal to 0, pitch(s) cannot do relative motion"""
		# =======================================================================
		self.Arm.relative_rot_nsa(pitch =  rot)  # pitch
		self.Arm.relative_rot_nsa(pitch = -rot)
		self.Arm.relative_rot_nsa(roll  =  rot)  # roll
		self.Arm.relative_rot_nsa(roll  = -rot)
		self.Arm.relative_rot_nsa(yaw   =  rot)  # yaw
		self.Arm.relative_rot_nsa(yaw   = -rot)

	def test_relative_xyz_base(self, dis = 0):
		self.Arm.relative_xyz_base(x =  dis)
		self.Arm.relative_xyz_base(x = -dis)
		self.Arm.relative_xyz_base(y = -dis)
		self.Arm.relative_xyz_base(y =  dis)
		self.Arm.relative_xyz_base(z =  dis)
		self.Arm.relative_xyz_base(z = -dis)

	def test_relative_move_nsa_rot_pry(self, dis = 0, rot = 0):
		self.Arm.relative_move_nsa_rot_pry(n =  dis, pitch =  rot)
		self.Arm.relative_move_nsa_rot_pry(n = -dis, pitch = -rot)

		self.Arm.relative_move_nsa_rot_pry(s = -dis, roll =  rot)
		self.Arm.relative_move_nsa_rot_pry(s =  dis, roll = -rot)

		self.Arm.relative_move_nsa_rot_pry(a =  dis, yaw =  rot)
		self.Arm.relative_move_nsa_rot_pry(a = -dis, yaw = -rot)

	def test_relative_move_xyz_rot_pry(self, dis = 0, rot = 0):
		self.Arm.relative_move_xyz_rot_pry(x =  dis, pitch =  rot)
		self.Arm.relative_move_xyz_rot_pry(x = -dis, pitch = -rot)

		self.Arm.relative_move_xyz_rot_pry(y = -dis, roll =  rot)
		self.Arm.relative_move_xyz_rot_pry(y =  dis, roll = -rot)

		self.Arm.relative_move_xyz_rot_pry(z = -dis, yaw =  rot)
		self.Arm.relative_move_xyz_rot_pry(z =  dis, yaw = -rot)


if __name__ == '__main__':
	rospy.init_node('strategy')

	try:
		s = Strategy()
		s.start() 

		# dis = 0.05
		# suction_ang = 0
		# gripper_suction_deg(suction_ang)
		# rospy.sleep(1)
		# s.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-180, -30, 0) )
		# rospy.sleep(1)
		# s.Arm.relative_move_suction('ptp', suction_ang, dis)
		# s.Arm.relative_move_nsa(a = 0.05) 
		# s.Arm.relative_move_nsa(n = 0.05) 



		# write_PickInfo_2_JSON()


		# ========== TEST ===========
		# Error pose
		#s.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.2), (-90, 0, 0) )
		#s.Arm.pub_ikCmd('ptp', (0.40, 0.00 , 0.15), (-180, 0, 0))
		
		
		#s.Arm.relative_move_nsa(a = 0.15) 
		# s.Arm.pub_ikCmd('ptp', (x, y , z), (pitch, roll, yaw) )
		
		#s.arm_go_init_pose()

		#s.arm_go_init_pose()
		
		# s.test_go_bin_LM('g')



		#s.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.5), (-120, 0, 0) )
			
		#s.arm_bin_photo()
			
		# s.stow.LM_2_tote()			    # -
		
		# # s.safe_pose()

		# #s.arm_bin_photo()
		# #gripper_vaccum_off()
		# #gripper_suction_up()
		# #gripper_suctoin_down()
		
		# s.stow.arm_photo_pose()	
		# exit()
		#s.Arm.home()
		#s.stow.test_obj_pose('robots_dvd')  

		
		#s.stow.test_run_with_obj_pose(-0.052098851651, -0.0093888239935, 0.554290473461)
		#s.stow.test_run_with_obj_pose(-0.0642712190747, 0.0477893352509, 0.546670496464)	
		
		
		#s.Arm.relative_move_nsa(n = 0.04, s = -0.06, a = 0.1)  
		#s.Arm.relative_move_nsa(s = -0.06)  #s= move_cam_x




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
