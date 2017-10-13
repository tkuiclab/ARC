#! /usr/bin/env python
# pylint: disable = invalid-name
# pylint: disable = C0326, C0121, C0301
# pylint: disable = W0105, C0303, W0312
"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos, pi
from numpy import multiply
import numpy

import rospy
import roslib; #roslib.load_manifest('obj_pose')
import tf

import actionlib
from std_msgs.msg import String, Float64

import obj_pose.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import json

import arm_task_rel
from gripper import *
from std_msgs.msg import String, Bool
from strategy.srv import *

gripper_srv = None
busy = None

class T2O:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        #self.__set_pubSub()
        self.Arm 			= arm_task_rel.ArmTask('/left_arm')
        rospy.on_shutdown(self.stop_task)
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        # rospy.wait_for_service('gripper_cmd')
        global gripper_srv
        gripper_srv = rospy.ServiceProxy('gripper_cmd', Gripper)
        # self.stop_gripper_motor()
        # self.open_finger()
        # self.close_hand()

    def stop_task(self):
        """Stop task running."""
        self.Arm.stop_task()
        
    #request object pose
    def obj_pose_request(self, obj):
        while self.Arm.busy:
            rospy.sleep(.1)
        rospy.loginfo('obj_pose_request() obj='+obj)
        
        goal = obj_pose.msg.ObjectPoseGoal(
            object_name=obj,
            limit_ary = [-0.13, 0.13, -0.15,  0.3, 0.3, 0.63])

        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

    
    def safe_pose(self):
        # self.Arm.pub_ikCmd('ptp', (0.20, 0.00 , 0.3), (-150, 0, 0))
        self.Arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)


    def robot_photo_pose(self):
        # self.Arm.pub_ikCmd('ptp', (0.2, 0.00 , 0.45), (-150, 0, 0))
        # self.Arm.pub_ikCmd('ptp', (0.45, 0.00 , 0.15), (180, 0, 0))
        # self.Arm.pub_ikCmd('ptp', (0.3, 0.00 , 0.15), (180, 0, 0))
        self.Arm.pub_ikCmd('ptp', (0.45, 0.00 , 0.25), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)

    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

    def close_finger(self):
        # self.stop_gripper_motor()
        gripper_srv('4')

    
    def open_finger(self):
        self.stop_gripper_motor()
        gripper_srv('2')


    def open_hand(self):
        self.stop_gripper_motor()
        gripper_srv('10')


    def close_hand(self):
        self.stop_gripper_motor()
        gripper_srv('8')

    def stop_gripper_motor(self):
        gripper_srv('0')
        
    def obj_pose_done_cb(self, state, result):
        # self.safe_pose()
        rospy.loginfo(result.object_pose)
        task.Arm.relative_rot_nsa(pitch = result.object_pose.angular.x)
        task.Arm.relative_rot_nsa(yaw = result.object_pose.angular.y)
        task.Arm.relative_rot_nsa(roll = result.object_pose.angular.z)
        # task.Arm.relative_move_nsa(a=-0.1)
        # self.open_hand()
        # task.Arm.Move2_Abs_xyz(x = result.object_pose.linear.x, y=result.object_pose.linear.y*-1, z=0.3)
        task.Arm.Move2_Abs_xyz(x = result.object_pose.linear.x, y=result.object_pose.linear.y*-1, z=0.15)

        # gripp_z = 0.3
        # if result.object_pose.linear.z < 0.03:
        #     gripp_z = 0.03
        # else:
        #     gripp_z = result.object_pose.linear.z
        gripp_z = result.object_pose.linear.z
        print gripp_z
        task.Arm.Move2_Abs_xyz(x = result.object_pose.linear.x, y=result.object_pose.linear.y*-1, z=gripp_z)   
        # task.Arm.Move2_Abs_xyz(x = result.object_pose.linear.x, y=result.object_pose.linear.y*-1, z=0.15)  
        # self.close_finger()
        task.Arm.pub_ikCmd('ptp', (0.20, 0.00 , 0.4), (-150, 0, 0))
        # self.open_gripper()

  


if __name__ == '__main__':

    rospy.init_node('t2o_robot', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    rospy.loginfo('T2O Ready')
    task.safe_pose()
    task.robot_photo_pose()
    # task.obj_pose_request('hand_weight')
    # task.obj_pose_request('crayons')
    # task.obj_pose_request('marbles')
    task.obj_pose_request("bath_sponge")
    # task.obj_pose_request('duct_tape')

    while task.Arm.busy:
        rospy.sleep(.1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
