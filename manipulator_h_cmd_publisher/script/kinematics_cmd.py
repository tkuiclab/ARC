#!/usr/bin/env python

import rospy
import tf
import math
import random

# import msgs
from std_msgs.msg import String
from manipulator_h_base_module_msgs.msg import KinematicsPose
from manipulator_h_base_module_msgs.srv import GetKinematicsPose
from robotis_controller_msgs.msg import StatusMsg

class pose_type(object):
    def __init__(self, position = [0.0, 0.0, 0.0], euler = [0.0, 0.0, 0.0], quater = [0.0, 0.0, 0.0, 0.0]):
    	self.position    = position
        self.euler_angle = euler            # r, p, y
        self.quaternion  = quater           # x, y, z, w
        self.__flg_euler_deg2rad = False
        self.__flg_euler_rad2deg = False
    def euler_deg2rad(self):
        if not self.__flg_euler_deg2rad:
            self.euler_angle[0] = math.radians(self.euler_angle[0])
            self.euler_angle[1] = math.radians(self.euler_angle[1])
            self.euler_angle[2] = math.radians(self.euler_angle[2])
            self.__flg_euler_deg2rad = True
            self.__flg_euler_rad2deg = False
    def euler_rad2deg(self):
        if not self.__flg_euler_rad2deg:
            self.euler_angle[0] = math.degrees(self.euler_angle[0])
            self.euler_angle[1] = math.degrees(self.euler_angle[1])
            self.euler_angle[2] = math.degrees(self.euler_angle[2])
            self.__flg_euler_deg2rad = False
            self.__flg_euler_rad2deg = True
    def e2q(self):
        qr = tf.transformations.quaternion_from_euler(
            self.euler_angle[0],
            self.euler_angle[1],
            self.euler_angle[2])
        self.quaternion = [qr[0], qr[1], qr[2], qr[3]]
    def q2e(self):
        euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.euler_angle = [euler[0], euler[1], euler[2]]

# class kinematics_cmd_type(object):
#     def __init__(self):
#     	self.position    = [0.0, 0.0, 0.0]
#         self.euler_angle = [0.0, 0.0, 0.0]          # r, p, y
#         self.quaternion  = [0.0, 0.0, 0.0, 0.0]     # x, y, z, w
#         self.__flg_euler_deg2rad = False
#         self.__flg_euler_rad2deg = False
#     def euler_deg2rad(self):
#         if not self.__flg_euler_deg2rad:
#             self.euler_angle[0] = math.radians(self.euler_angle[0])
#             self.euler_angle[1] = math.radians(self.euler_angle[1])
#             self.euler_angle[2] = math.radians(self.euler_angle[2])
#             self.__flg_euler_deg2rad = True
#             self.__flg_euler_rad2deg = False
#     def euler_rad2deg(self):
#         if not self.__flg_euler_rad2deg:
#             self.euler_angle[0] = math.degrees(self.euler_angle[0])
#             self.euler_angle[1] = math.degrees(self.euler_angle[1])
#             self.euler_angle[2] = math.degrees(self.euler_angle[2])
#             self.__flg_euler_deg2rad = False
#             self.__flg_euler_rad2deg = True
#     def e2q(self):
#         qr = tf.transformations.quaternion_from_euler(
#             self.euler_angle[0],
#             self.euler_angle[1],
#             self.euler_angle[2])
#         self.quaternion = [qr[0], qr[1], qr[2], qr[3]]
#     def q2e(self):
#         euler = tf.transformations.euler_from_quaternion(self.quaternion)
#         self.euler_angle = [euler[0], euler[1], euler[2]]

class pose_cmd(object):
    def __init__(self):
    	self.__moving  = False
        self.__ik_fail = False
        rospy.init_node('kinematics_cmd', anonymous=True)

    	self.__pub_setMode = rospy.Publisher('/robotis/base/set_mode_msg', String, queue_size=0, latch=True)
    	self.__pub_iniPose = rospy.Publisher('/robotis/base/ini_pose_msg', String, queue_size=0, latch=True)
    	self.__pub_cmd = rospy.Publisher('/robotis/base/kinematics_pose_msg', KinematicsPose, queue_size=0, latch=True)
    	rospy.Subscriber('robotis/status', StatusMsg, self.__status_msg_callback)
        self.__srv_name = '/robotis/base/get_kinematics_pose'
        self.__srv_get_pose = rospy.ServiceProxy(self.__srv_name, GetKinematicsPose)

    def publish_cmd(self, msg):
        self.__set_moving()
        self.__pub_cmd.publish(msg)

    def run_random(self):
    	self.set_mode()
        self.initial_pose()
    	rate = rospy.Rate(10)

    	while not rospy.is_shutdown():
            if not self.arm_is_moving():
                self.publish_cmd(self.__generate_cmd())
    	    rate.sleep()

    def set_mode(self):
        self.__pub_setMode.publish('set_mode')
    	rospy.loginfo('Set base module')

    def initial_pose(self):
        self.__set_moving()
        self.__pub_iniPose.publish('ini_pose')
        rospy.loginfo('Go initial pose')

    def __generate_cmd(self):
        cmd = pose_type()
        cmd.position    = [
            random.uniform(0.35,  0.45),
            random.uniform(-0.10, 0.10),
            random.uniform(0.15,  0.35)]  # x, y, z
        cmd.euler_angle = [
            random.uniform(-180, 180),
            random.uniform(-90, 90),
            random.uniform(-60, 60)]  # roll, pitch, yaw
        return self.cmd_to_kinematics_msg(cmd)

    def cmd_to_kinematics_msg(self, cmd):
        cmd_msg = KinematicsPose()
        cmd_msg.name = "arm"
        cmd_msg.pose.position.x = cmd.position[0]
        cmd_msg.pose.position.y = cmd.position[1]
        cmd_msg.pose.position.z = cmd.position[2]
        # euler to quaternion
        cmd.euler_deg2rad()
        cmd.e2q()
        cmd_msg.pose.orientation.x = cmd.quaternion[0];
        cmd_msg.pose.orientation.y = cmd.quaternion[1];
        cmd_msg.pose.orientation.z = cmd.quaternion[2];
        cmd_msg.pose.orientation.w = cmd.quaternion[3];
        return cmd_msg

    def get_pose(self, display=False):
        try:
            rospy.wait_for_service(self.__srv_name)
            resp = self.__srv_get_pose('arm')
            pose = pose_type()
            pose.position = [
                resp.group_pose.position.x,
                resp.group_pose.position.y,
                resp.group_pose.position.z]
            pose.quaternion = [
                resp.group_pose.orientation.x,
                resp.group_pose.orientation.y,
                resp.group_pose.orientation.z,
                resp.group_pose.orientation.w]
            pose.q2e()

            if display:
                pose.euler_rad2deg()
                rospy.loginfo('x: ' + str(pose.position[0]) + ' y: ' + str(pose.position[1]) + ' z: ' + str(pose.position[2]))
                rospy.loginfo('r: ' + str(pose.euler_angle[0]) + ' p: ' + str(pose.euler_angle[1]) + ' y: ' + str(pose.euler_angle[2]))
                pose.euler_deg2rad()

            return pose
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s', e)

    def __set_moving(self):
        self.__moving = True

    def arm_is_moving(self):
        return self.__moving

    def is_ik_fail(self):
        return self.__ik_fail

    def __status_msg_callback(self, msg):
    	rospy.loginfo(self.log_level(msg.type) + msg.module_name + ' ' + msg.status_msg)
    	if 'End Trajectory' in msg.status_msg:
            self.__moving = False
            if 'IK Failed' in msg.status_msg:
                rospy.logwarn('IK fail')
                self.__ik_fail = True

    def log_level(self, type):
        if   1 == type:
    	    return '[INFO] '
    	elif 2 == type:
        	return '[WARN] '
    	elif 3 == type:
        	return '[ERROR] '
    	return '[UNKNOW] '

if __name__ == '__main__':
    try:
        cmd = pose_cmd()
        cmd.run_random()
    except rospy.ROSInterruptException:
        rospy.loginfo('Pushiler is closed')
