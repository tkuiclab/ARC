#!/usr/bin/env python


import roslib; roslib.load_manifest('move_base')
import rospy
import rospkg
import roslaunch


from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import Char
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from strategy.srv import *

import tf
import threading
import vision.msg
import time
import math




face_fire_ang = 4		#Face Mode, Fire Angle Tolerace
close_fire_ang = 20		#Close Mode, Fire Angle Tolerace Ang
extinguish_fire_ang = 4	#Extinguish Mode, , Fire Angle Tolerace Ang

#fire_dis_ary = [2.05, 3.1, 2.5]			#for fire_dis with 0cm, 50cm, 100cm
fire_dis_ary = [2.5, 3.3, 2.5]			#for fire_dis with 0cm, 50cm, 100cm
fire_dis = 3.5			#   fire_dis - fire_dis_tolerance <   push extinguisher < 	fire_dis + fire_dis_tolerance
fire_dis_tolerance = 0.08

special_fire_dis_25 = 2.25
special_tilt_want_25 = 0.42


Tilt_Mid_Ary = [0.45, 0.1, -0.2]		#for Tilt_Mid with 0cm, 50cm, 100cm
Tilt_Mid = 0.1
After_Extinguished_WaitTime = 300	#300s (5min)

Robot_Speed = 50 		#for vision dis check

Max_Speed_XY = 2.5
Max_Speed_Yaw = 12.56

SeekMode  				= 1			# Seek(find) fire
FaceMode  				= 2			# Face fire
CheckHeightMode  		= 3			# Check fire height
CloseMode 				= 4			# Close fire
ExtinguishMode 			= 5			# Push Extinguisher
AfterExtinguishMode 	= 6			# Push Extinguisher
PatrolMode				= 7 		# Patrol
SmokeDetectorMode		= 8 		# Go Target, SmokeDetectorMode
StandbyMode				= 9 		# Standby Mode, stop robot, you can manual control robot in this mode
LosePosMode				= 10		# if move_base fail, it will switch to this mode
GoPointMode				= 11		# Go specific point(pose) and switch to StandbyMode



Patrol_Point_1			= 1
Patrol_Point_2			= 2
Patrol_Point_3			= 3
Patrol_Point_4			= 4
Patrol_Point_5   	    = 5
Patrol_Point_6          = 6
Patrol_Point_7          = 7

#move_base result
MOVE_BASE_PENDING	=	0
MOVE_BASE_ACTIVE	=	1
MOVE_BASE_PREEMPTED	=	2
MOVE_BASE_SUCCEEDED	=	3
MOVE_BASE_ABORTED	=	4
MOVE_BASE_REJECTED	=	5
MOVE_BASE_PREEMPTING=	6
MOVE_BASE_RECALLING	=	7
MOVE_BASE_RECALLED	=	8
MOVE_BASE_LOST		=	9

class Strategy(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		self.var_init()
		self.launch_hector()

		#------------ROS pub, subscribe, shutdown ------------#
		#motion control
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		#tilt control
		self.tilt_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
		self.liner_motor_pub = rospy.Publisher('/linear_motor', Bool, queue_size=10)


		#thermal vision
		rospy.Subscriber("FireStatus", vision.msg.FireStatus, self.vision_cb)
		#for tell thermal processing, the odometry stae
		self.robot_state_pub = rospy.Publisher('/robot/state', Char, queue_size=10)

		rospy.Subscriber("/smoke_detctor", Twist, self.smoke_detctor_cb)

		#odom subscriber
		#rospy.Subscriber('odom',Odometry, self.odometry_cb)
		rospy.Service('/robot_cmd', RobotCmd, self.robot_cmd_cb)


		rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.amcl_pose_cb)


		self.robot_mode_pub = rospy.Publisher('/robot_mode', String, queue_size=3)

		#-------move_base------#
		# rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
		self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=3)


		#Move_base Action Client
		rospy.loginfo("Wait Move_Base Action....")
		self.move_base_act = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base_act.wait_for_server()


		#ros shutdown
		rospy.on_shutdown(self.shutdown)


		rospy.loginfo("Strategy Ready!!")


	def var_init(self):
		self.speed = 30
		self.fire_status = vision.msg.FireStatus()

		self.face_close_fire_flag = False
		self.face_close_fire_times = 0
		self.extinguish_fire_step = 1
		self.tilt_val = 0.5			#tile_val switch 0.5 -0.8 0.5 -0,8
		self.tilt_pre_time = 0
		self.line_motor_pub_num = 0

		#for get Ctrl+C
		#signal.signal(signal.SIGINT, self.signal_handler)
		self.fire_happen = False  		# if fire_happen, have extinguished fire(no fire) -> stay up to check fire
		self.shake_detect = False
		self.fire_status_time = -1


		#-------for check fire height----------#
		self.fire_height_check_0 = False		#for send '0' to thermal process
		self.fire_height_check_0_x = -9999
		self.fire_height_check_1 = False       #for send '1' to thermal process
		self.fire_height_check_finish = False
		self.fire_height_check_count = 0
		self.fire_height = -1
		self.is_reset_odom = False
		self.is_reset_odom_times = 0


		self.stop_robot = False

		self.tilt_want = 0.7
		self.fire_dis_want = 2.5

		#-------Patrol----------#
		self.patrol_run = False
		self.patrol_step = Patrol_Point_1
		self.move_base_status = MOVE_BASE_ACTIVE

		self.move_base_retry_time = 0

		#-------SmokeDetectorMode----------#
		self.smoke_detector_go_target = False

		#-------LosePose----------#
		self.losepose_wait_launch_pre_time = 0
		self.losepose_pre_mode = StandbyMode
		self.losepose_amcl_ready = False

		self.pre_mode = None
		self.mode = StandbyMode
		#self.mode = LosePosMode
		self.standymode_stop_times = 0


	def shutdown(self):
		self.stop_robot = True
		self.extinguish_push(False)
		self.pub_motion(0, 0, 0)
		rospy.loginfo("Strategy Exit & Stop Robot")


	def robot_cmd_cb(self,req):
		cmd = req.cmd
		rospy.loginfo("Request cmd = " + req.cmd)



		if cmd=="Patrol":
			self.patrol_run = False
			self.mode = PatrolMode
		elif cmd=="Standby":
			self.standymode_stop_times = 0
			self.mode = StandbyMode
		elif cmd=="GoPoint":
			self.go_specific_point(req.pose.linear.x,req.pose.linear.y, req.pose.angular.z )
			self.mode = GoPointMode


		res = RobotCmdResponse()
		res.success = True
		res.msg = "Success"
		return res

	def amcl_pose_cb(self,d):
		if self.mode == LosePosMode:
			tolerace = 0.1
			if(d.pose.covariance[0] < tolerace and d.pose.covariance[7] < tolerace):
				self.losepose_amcl_ready = True
				rospy.loginfo("/amcl_pose cov[0] = " + str(d.pose.covariance[0]))
				rospy.loginfo("/amcl_pose cov[7] = " + str(d.pose.covariance[7]))


	def vision_cb(self,data):
		#if not self.shake_detect:
		if data.fire_detect and data.dis!=999:
			if  self.fire_status_time == -1:
				self.fire_status = data
				self.fire_status_time = time.time()
			else:
				sub_time = time.time() - self.fire_status_time
				sub_dis = abs(data.dis - self.fire_status.dis)
				if sub_dis < (Robot_Speed * sub_time):
					self.fire_status = data
					self.fire_status_time = time.time()
				else:
					#get angle only
					self.fire_status.ang = data.ang
					rospy.logwarn("vision_cb false dis = " + str(data.dis)+",pre_dis="+ str(self.fire_status.dis)+",sub_time="+ str(sub_time))
		else:
			self.fire_status_time = -1
			self.fire_status = data

	def smoke_detctor_cb(self, cmd):
		self.mode = SmokeDetectorMode
		self.smoke_detector_go_target = False

		q = tf.transformations.quaternion_from_euler(0, 0, cmd.angular.z)

		#set goal
		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = cmd.linear.x
		goal.target_pose.pose.position.y = cmd.linear.y
		goal.target_pose.pose.orientation.x = q[0]
		goal.target_pose.pose.orientation.y = q[1]
		goal.target_pose.pose.orientation.z = q[2]
		goal.target_pose.pose.orientation.w = q[3]
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()


		#self.move_base_act.send_goal(goal, done_cb = self.move_base_act_done_cb, feedback_cb=self.move_base_act_feedback_cb)
		self.move_base_act.send_goal(goal, done_cb = self.move_base_act_done_cb)

		#set again
		self.mode = SmokeDetectorMode

		rospy.loginfo("smoke_detctor_cb() go (x, y, yaw)=(" + str(cmd.linear.x) + ", " + str(cmd.linear.y) + ", " + str(cmd.angular.z) +  ")")

	def move_base_act_done_cb(self, status, result):
		#rospy.loginfo("move_base_result_cb) status=" + str(status.status) + ", text = "+  status.text)
		#if res.status.status == MOVE_BASE_SUCCEEDED:
		#rospy.loginfo("move_base_act_done_cb() status=" + str(status) )

		if status == MOVE_BASE_PREEMPTED:
			#trigger by cancel goal
			rospy.logwarn("move_base_act_done_cb() say status=MOVE_BASE_PREEMPTED")
			return

		if status != MOVE_BASE_SUCCEEDED:
			self.move_base_retry_time = self.move_base_retry_time + 1
			rospy.logwarn("move_base_act_done_cb() say != SUCCEEDED  status=" + self.str_move_base_status(status) )
			rospy.logwarn("move_base retry times=" + str(self.move_base_retry_time))

			if self.move_base_retry_time > 0:
				self.launch_hector_amcl()
				self.losepose_wait_launch_pre_time = rospy.get_time()
				self.losepose_pre_mode = self.mode
				self.losepose_amcl_ready = False
				self.mode = LosePosMode
				self.move_base_retry_time  = 0
				self.patrol_run = False

		if self.mode == PatrolMode:
			if (self.patrol_step < Patrol_Point_7):
				self.patrol_step = self.patrol_step + 1
			elif (self.patrol_step == Patrol_Point_7) :
				self.patrol_step = Patrol_Point_1

			self.patrol_run = False
		elif self.mode == SmokeDetectorMode:
			self.smoke_detector_go_target = True
		elif self.mode == GoPointMode:
			self.mode = StandbyMode


	def str_move_base_status(self, status):
		if (status == MOVE_BASE_PENDING):
			return "MOVE_BASE_PENDING"
		elif (status == MOVE_BASE_ACTIVE):
			return "MOVE_BASE_ACTIVE"
		elif (status == MOVE_BASE_PREEMPTED):
			return "MOVE_BASE_PREEMPTED"
		elif (status == MOVE_BASE_SUCCEEDED):
			return "MOVE_BASE_SUCCEEDED"
		elif (status == MOVE_BASE_ABORTED):
			return "MOVE_BASE_ABORTED"
		elif (status == MOVE_BASE_REJECTED):
			return "MOVE_BASE_REJECTED"
		elif (status == MOVE_BASE_PREEMPTING):
			return "MOVE_BASE_PREEMPTING"
		elif (status == MOVE_BASE_RECALLING):
			return "MOVE_BASE_RECALLING"
		elif (status == MOVE_BASE_RECALLED):
			return "MOVE_BASE_RECALLED"
		elif (status == MOVE_BASE_LOST):
			return "MOVE_BASE_LOST"



	def str_mode(self, status):
		if (status == SeekMode):
			return "SeekMode"
		elif (status == FaceMode):
			return "FaceMode"
		elif (status == CheckHeightMode):
			return "CheckHeightMode"
		elif (status == CloseMode):
			return "CloseMode"
		elif (status == ExtinguishMode):
			return "ExtinguishMode"
		elif (status == AfterExtinguishMode):
			return "AfterExtinguishMode"
		elif (status == PatrolMode):
			return "PatrolMode"
		elif (status == SmokeDetectorMode):
			return "SmokeDetectorMode"
		elif (status == StandbyMode):
			return "StandbyMode"
		elif (status == LosePosMode):
			return "LosePosMode"
		elif (status == GoPointMode):
			return "GoPointMode"

	# def move_base_result_cb(self,res):
	#
	# 	rospy.loginfo("(move_base_result_cb) status=" + str(res.status.status) + ", text = "+  res.status.text)
	# 	#if res.status.status == MOVE_BASE_SUCCEEDED:
	# 	if self.mode == PatrolMode:
	# 		if (self.patrol_step == Patrol_Point_1 or
	# 			self.patrol_step == Patrol_Point_2 or
	# 			self.patrol_step == Patrol_Point_3 ):
	# 			self.patrol_step = self.patrol_step + 1
	# 		elif (self.patrol_step == Patrol_Point_3) :
	# 			self.patrol_step = Patrol_Point_1
	#
	# def move_base_status_cb(self,res):
	# 	rospy.logwarn("move_base_status_cb()  move_base_status= " + str(self.move_base_status) )
	#
	# 	if len(res.status_list) > 0 :
	# 		self.move_base_status = res.status_list[0].status
	# 		if self.move_base_status != MOVE_BASE_ACTIVE and self.move_base_status != MOVE_BASE_PREEMPTED  :
	# 			rospy.logwarn("move_base_status_cb()  move_base_status= " + str(self.move_base_status) )

				#rospy.loginfo("res.status_list.length=" + str(len(res.status_list)) )

	# def odometry_cb(self, msg):
	# 	self.robot_position = msg.pose.pose.position

	def set_speed(self,v):
		_speed = v


	def pub_motion(self,x, y, yaw):
		cmd = Twist()
		cmd.linear.x = x * self.speed /100.0 * Max_Speed_XY
		cmd.linear.y = y * self.speed /100.0 * Max_Speed_XY
		cmd.angular.z = yaw /100.0 * Max_Speed_Yaw

		#rospy.loginfo("set x =%lf, y=%lf, yaw=%lf",x,y,yaw);
		#rospy.loginfo("pub motion set x_speed =%lf, y_speed=%lf, yaw=%lf",cmd.linear.x, cmd.linear.y, cmd.angular.z);
		#if x!= 0 or y!=0 or yaw!=0:
		#	rospy.loginfo("set x_speed =%lf, y_speed=%lf, yaw=%lf", cmd.linear.x, cmd.linear.y, yaw);

		self.cmd_vel_pub.publish(cmd)

	def reset_odom(self):
		cmd = Twist()
		cmd.angular.x = 1.0
		rospy.loginfo("reset motion odom")

		self.cmd_vel_pub.publish(cmd)


	def tilt_control(self,val):
		#cmd = Float64()
		self.tilt_pub.publish(val)
		#rospy.loginfo("set tilt to %lf", val)

	#liner_motor run, True: push it, False:pull it
	def extinguish_push(self, yn):
		if self.line_motor_pub_num < 3:
			self.liner_motor_pub.publish(yn)
			self.line_motor_pub_num = self.line_motor_pub_num + 1
		#rospy.loginfo("set tilt to %lf", val)

	def fire_target_ang(self, target_ang):
		ang = self.fire_status.ang

		if(target_ang > 0 and ang < target_ang ):
			rospy.loginfo("fire_target_ang() ->  target_ang=" + str(target_ang) + ",fire_ang=" + str(ang))
			sub_ang = target_ang - ang
			self.pub_motion(0, 0, -sub_ang * 0.1)
			return False
		elif (target_ang < 0 and ang > target_ang):
			rospy.loginfo("fire_target_ang() -> target_ang=" + str(target_ang) + ",fire_ang=" + str(ang))
			sub_ang = target_ang - ang
			self.pub_motion(0, 0, -sub_ang * 0.1)
			return False
		else:
			rospy.loginfo("fire_target_ang() -> OK! target_ang=" + str(target_ang) + ",fire_ang=" + str(ang))
			return True
			#self.pub_motion(0, 0.5f, 0)


	def face_fire(self, tolerance, spd_multi):
		ang = self.fire_status.ang

		if(abs(ang) > tolerance ):
			#rospy.loginfo("face_fire() -> tolerace= " + str(tolerance) + ", ang=" + str(ang))
			self.pub_motion(0, 0, ang * spd_multi)
			return False
		else:
			#rospy.loginfo("face_fire() -> OK! tolerace= " + str(tolerance) + ", ang=" + str(ang))
			return True
			#self.pub_motion(0, 0.5f, 0)


	def close_fire(self):
		#return True
		dis = self.fire_status.dis
		ang = self.fire_status.ang



		if(dis > (self.fire_dis_want + fire_dis_tolerance) ):
			rospy.loginfo("(Close) close_fire() -> now dis =" + str(dis) + "ang = "+ str(ang))
			rospy.loginfo("(Close) self.fire_dis_want= "+ str(self.fire_dis_want))

			diff = dis - self.fire_dis_want
			if diff < 1 :
				diff = 1

			#modify speed / 5 -> 30 / 2 = 15
			#v_x = (diff )*0.4 * math.sin(ang * math.pi / 180)
			#v_y = (diff )*0.4 * math.cos(ang * math.pi / 180)

			v_y = (-1)*(diff )*0.4 * math.sin(ang * math.pi / 180)
			v_x = (diff )*0.4 * math.cos(ang * math.pi / 180)

			#self.pub_motion(v_x, v_y, ang*0.01)
			self.pub_motion(v_x, 0, 0)
			return False
		elif(dis < (self.fire_dis_want - fire_dis_tolerance) ):
			rospy.loginfo("(Close_Back) BACK -> now dis =" + str(dis) + "ang = "+ str(ang))
			rospy.loginfo("(Close_Back) self.fire_dis_want= "+ str(self.fire_dis_want))
			#v_x = (dis - fire_dis )*0.4 * math.sin(ang * math.pi / 180)
			#v_y = (dis - fire_dis )*0.4 * math.cos(ang * math.pi / 180)

			v_y = (-1)*(dis - self.fire_dis_want )*0.4 * math.sin(ang * math.pi / 180)
			v_x = (dis - self.fire_dis_want )*0.4 * math.cos(ang * math.pi / 180)


			#self.pub_motion( -v_x, -v_y, ang*0.1)
			#self.pub_motion( v_x, v_y, ang*0.01)
			self.pub_motion(v_x, 0, 0)
			return False
		else:
			return True

	def tilt_test(self):
		self.tilt_control(Tilt_Mid)



	def extinguish_fire(self):
		self.extinguish_push(True)
		#self.tilt_run()
		#self.tilt_control(Tilt_Mid)
		rospy.loginfo("(Extinguish_fire) self.tilt_want="+str(self.tilt_want) + "fire_height="+str(self.fire_height))
		self.tilt_control(self.tilt_want)
		if(self.face_fire(3,0.001)):
			self.pub_motion(0, 0, 0)

	def update_fire_dis_height_want(self):
		self.fire_height = self.fire_status.height
		if(self.fire_height == 0):
			self.speed = 20
			self.fire_dis_want = fire_dis_ary[0]
			self.tilt_want = Tilt_Mid_Ary[0]
		#elif(self.fire_height == 25):
		#	self.speed = 50
		#	self.fire_dis_want = special_fire_dis_25
		#	self.tilt_want = special_tilt_want_25
		elif(self.fire_height == 50):
			self.speed = 10
			self.fire_dis_want = fire_dis_ary[1]
			self.tilt_want = Tilt_Mid_Ary[1]
		elif(self.fire_height == 100):
			self.speed = 10
			self.fire_dis_want = fire_dis_ary[2]
			self.tilt_want = Tilt_Mid_Ary[2]
		else:
			self.speed = 10
			self.fire_dis_want = 3.8
			self.tilt_want = Tilt_Mid_Ary[1]

	def patrol_mode(self):

		#if self.move_base_status == MOVE_BASE_ACTIVE or self.move_base_status ==  MOVE_BASE_SUCCEEDED:
		want_x = 0
		want_y = 0
		want_yaw = 0

		# if self.patrol_step == Patrol_Point_1:   #1
		#     want_x = 0.5
		#     want_y = 5.0
		#     want_yaw = -90   #degree
		#
		# elif self.patrol_step == Patrol_Point_2: #2
		# 	want_x = 4.0
		# 	want_y = 1.5
		# 	want_yaw = 45	#degree
		# elif self.patrol_step == Patrol_Point_3: #3
		#     want_x = 4
		#     want_y = 5
		#     want_yaw = -90
		# elif self.patrol_step == Patrol_Point_4: #2
		# 	want_x = 4
		# 	want_y = 1.5
		# 	want_yaw = 90
		# elif self.patrol_step == Patrol_Point_5: #4
		# 	want_x = 6.5
		# 	want_y = 5
		# 	want_yaw = -45
		# elif self.patrol_step == Patrol_Point_6: #5
		# 	want_x = 6.55 #9
		# 	want_y = -0.3 #1.5
		# 	want_yaw = 0 #90
		# elif self.patrol_step == Patrol_Point_7: #6
		# 	# want_yaw = 90
		# 	want_x = 9.6 #9.5
		# 	want_y = 2.7 #4
		# 	want_yaw = 45 #-90
		# else:
		# 	rospy.logwarn("patrol_mode() say FAIL self.patrol_step = "+self.patrol_step)
		#### Husky Field####
		if self.patrol_step == Patrol_Point_1:   #1
			want_x = 3.0
			want_y = -6.0
			want_yaw = 90   #degree

		elif self.patrol_step == Patrol_Point_2: #2
			want_x = -3.0
			want_y = -6.0
			want_yaw = -90	#degree
		elif self.patrol_step == Patrol_Point_3: #3
			want_x = 0.0
			want_y = 0.0
			want_yaw = 0
		elif self.patrol_step == Patrol_Point_4: #2
			want_x = 3.0
			want_y = 6.0
			want_yaw = 90   #degree

		elif self.patrol_step == Patrol_Point_5: #4
			want_x = -3.0
			want_y = 6.0
			want_yaw = 90   #degree

		elif self.patrol_step == Patrol_Point_6: #5
			want_x = 0.0
			want_y = 0.0
			want_yaw = 0

		elif self.patrol_step == Patrol_Point_7: #6
			# want_yaw = 90
			want_x = -3.0
			want_y = 6.0
			want_yaw = 90   #degree

		else:
			rospy.logwarn("patrol_mode() say FAIL self.patrol_step = "+self.patrol_step)


		rospy.loginfo("(Patrol) Go Next Point ->  " + str(self.patrol_step ) )


		q = tf.transformations.quaternion_from_euler(0, 0, math.radians(want_yaw) )


		#set goal
		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = want_x
		goal.target_pose.pose.position.y = want_y
		goal.target_pose.pose.orientation.x = q[0]
		goal.target_pose.pose.orientation.y = q[1]
		goal.target_pose.pose.orientation.z = q[2]
		goal.target_pose.pose.orientation.w = q[3]
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()


		#self.move_base_act.send_goal(goal, done_cb = self.move_base_act_done_cb, feedback_cb=self.move_base_act_feedback_cb)
		self.move_base_act.send_goal(goal, done_cb = self.move_base_act_done_cb)

		self.patrol_run = True


	def go_specific_point(self,want_x, want_y, want_yaw):
		rospy.loginfo("go_specific_point() say go (%lf,%lf,%lf)",want_x,want_y,want_yaw)
		q = tf.transformations.quaternion_from_euler(0, 0, want_yaw )

		#set goal
		goal = MoveBaseGoal()
		goal.target_pose.pose.position.x = want_x
		goal.target_pose.pose.position.y = want_y
		goal.target_pose.pose.orientation.x = q[0]
		goal.target_pose.pose.orientation.y = q[1]
		goal.target_pose.pose.orientation.z = q[2]
		goal.target_pose.pose.orientation.w = q[3]
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()

		self.move_base_act.send_goal(goal, done_cb = self.move_base_act_done_cb)

	def launch_hector(self):
		try:
			self.hector_launch.shutdown() # does a exist in the current namespace
		except AttributeError:
			rospy.logwarn("self.hector_launch not defined")



		#pkg_name = "febot_slam_only_laser"
		pkg_name = "husky_only_laser"
		launch_name = "all_norviz.launch"
		pkg_path = rospkg.RosPack().get_path(pkg_name)
		full_path = pkg_path + "/launch/" + launch_name


		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		self.hector_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_path])
		self.hector_launch.start()



	def launch_hector_amcl(self):
		try:
			self.hector_launch.shutdown() # does a exist in the current namespace
		except AttributeError:
			rospy.logwarn("self.hector_launch not defined")



		#pkg_name = "febot_slam_only_laser"
		pkg_name = "husky_only_laser"
		launch_name = "all_only_laser_amcl_hector.launch"
		pkg_path = rospkg.RosPack().get_path(pkg_name)
		full_path = pkg_path + "/launch/" + launch_name


		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		self.hector_launch = roslaunch.parent.ROSLaunchParent(uuid, [full_path])
		self.hector_launch.start()



	def core(self):
		#self.extinguish_fire()
		# rate.sleep()
		#self.tilt_test()
		#return
		#if self.fire_target_ang(15):
		#	self.pub_motion(0, 0, 0)
		#print time.time()
		#self.extinguish_push()
		#self.tilt_control(Tilt_Mid)
		#return

		if self.stop_robot == True:
			return

		self.update_fire_dis_height_want()
		#rospy.loginfo("(CheckHeight) fire_dis = " + str(self.fire_dis_want) + ", tilt_want = " + str(self.tilt_want) + "fire_height="+str(self.fire_height))


		if self.pre_mode != self.mode:
			rospy.loginfo("Switch to Mode [" + self.str_mode(self.mode) +"]")
			self.pre_mode = self.mode


		self.robot_mode_pub.publish(self.str_mode(self.mode))


		if self.mode == StandbyMode:
			if self.standymode_stop_times < 10:
				g = GoalID()
				self.move_base_cancel_pub.publish(g)
				self.pub_motion(0,0,0)
				self.standymode_stop_times = self.standymode_stop_times + 1

		elif self.mode == PatrolMode:
			if self.patrol_run ==False:
				self.patrol_mode()
		elif self.mode == GoPointMode:
			#do nothing in GoPointMode
			return
		elif self.mode == LosePosMode:
			if(  ( rospy.get_time()  - self.losepose_wait_launch_pre_time ) > 3.0 ):	#wait start completely
				rospy.loginfo("(LosePosMode) Rotation")
				self.pub_motion(0, 0, 5)
				if self.losepose_amcl_ready == True:
					rospy.loginfo("(LosePosMode) losepose_amcl_ready, go pre_mode = " + str(self.losepose_pre_mode))
					self.pub_motion(0, 0, 0)
					self.mode = self.losepose_pre_mode

		elif self.mode == SmokeDetectorMode:
			if self.smoke_detector_go_target == True:
				self.mode = SeekMode
		elif self.mode == SeekMode:
			if (self.fire_status.fire_detect):
				self.mode = FaceMode
			else:
				rospy.loginfo("(Seek) Rotation")
				self.pub_motion(0, 0, 3)		#rotation
		elif self.mode == FaceMode:
			rospy.loginfo("fire_detect="+str(self.fire_status.fire_detect))
			if (self.fire_status.fire_detect==False):
				self.mode = SeekMode
				#self.mode == FaceMode
			else:
				rospy.loginfo("(Face) tolerace= " + str(face_fire_ang) + ", ang=" + str(self.fire_status.ang))
				if (self.face_fire(face_fire_ang, 0.01)):
					#self.mode = CloseMode
					self.mode = CloseMode
		elif self.mode == CheckHeightMode:
			#if (self.fire_status.fire_detect==False):
			#	#self.mode = SeekMode
			#	self.mode == CheckHeightMode
			#else:
			#	if(self.check_fire_height()):
			self.fire_height = self.fire_status.height

			if(self.fire_height == 0):
				#fire_dis = fire_dis_ary[0]
				#Tilt_Mid = Tilt_Mid_Ary[0]
				self.fire_dis_want = fire_dis_ary[0]
				self.tilt_want = Tilt_Mid_Ary[0]
			elif(self.fire_height == 50):
				self.fire_dis_want = fire_dis_ary[1]
				self.tilt_want = Tilt_Mid_Ary[1]
			else:
				self.speed = 10
				self.fire_dis_want = fire_dis_ary[2]
				self.tilt_want = Tilt_Mid_Ary[2]

			#self.tilt_control(Tilt_Mid)

			rospy.loginfo("(CheckHeightMode) Fire Height Check Finish! Fire Height = " + str(self.fire_height))
			rospy.loginfo("(CheckHeightMode) fire_dis = " + str(self.fire_dis_want) + ", tilt_want = " + str(self.tilt_want))
			self.fire_height_check_finish = True
			self.mode = CloseMode

		elif self.mode == CloseMode:
			if (self.fire_status.fire_detect==False):
				#self.mode = SeekMode
				self.mode = CloseMode
			elif (abs(self.fire_status.ang) > close_fire_ang ):
				self.mode = FaceMode
			else:
				if (self.close_fire()):
					self.face_close_fire_times = self.face_close_fire_times + 1
					rospy.loginfo("(Close_Count) dis = "+ str(self.fire_status.dis) + ", Count 30 -> face_close_fire_times="+str(self.face_close_fire_times))
					if(self.face_close_fire_times >= 10):
						self.line_motor_pub_num = 0
						self.face_close_fire_times = 0
						self.mode = ExtinguishMode
		elif self.mode == ExtinguishMode:
			if(self.fire_happen == True and self.fire_status.fire_detect==False):
				self.mode = AfterExtinguishMode
				return

			if (self.face_fire(extinguish_fire_ang, 0.01)):
				self.fire_happen = True
				self.fire_happen_last_time = time.time()
				self.extinguish_fire()
				rospy.loginfo("(Extinguish_Push) Extinguishing fire!")
				#if (self.fire_status.fire_detect==False):
				#	self.mode = AfterExtinguishMode
			else:
				rospy.loginfo("(Extinguish_Face) tolerace= " + str(extinguish_fire_ang) + ", ang=" + str(self.fire_status.ang))
		elif self.mode == AfterExtinguishMode:
			rospy.loginfo("(AlreadyExtinguish) Already Extinguished fire!")
			# now have extinguished fire, back liner motor & stop robot
			self.pub_motion(0, 0, 0)		#stop & check
			#check and wiat for 5 min
			if (time.time() - self.fire_happen_last_time) > After_Extinguished_WaitTime:
				self.fire_happen = False



	def run(self):
		rate = rospy.Rate(30)  # 10hz
		while not rospy.is_shutdown():
			self.core()

			rate.sleep()




if __name__ == '__main__':
	rospy.init_node('strategy')

	try:
		s = Strategy()
		s.start()
		#s.extinguish_push()


		rospy.spin()
	except rospy.ROSInterruptException:
		pass
