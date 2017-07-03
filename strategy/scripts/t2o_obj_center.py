#! /usr/bin/env python
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
from gripper import cam2tool_y, cam2tool_z
import s



# cam2tool_y = -0.095  #cam axis
# cam2tool_z = 0.25 + 0.035


class T2O:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        #self.__set_pubSub()
        self.Arm 			= arm_task_rel.ArmTask()

        rospy.on_shutdown(self.stop_task)
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        
        
    def stop_task(self):
        """Stop task running."""
        self.Arm.stop_task()
        
    #request object pose
    def obj_pose_request(self, obj):
        while self.Arm.busy:
            rospy.sleep(.1)
        rospy.loginfo('obj_pose_request() obj='+obj)
        
        goal = obj_pose.msg.ObjectPoseGoal(obj)

        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

    
    def safe_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)


    def robot_photo_pose(self):
        
        self.Arm.pub_ikCmd('ptp', (0.45, 0.00 , 0.15), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)


    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

        
    def obj_pose_done_cb(self, state, result):
        self.arm_2_obj(result.object_pose)

    def arm_2_obj(self, obj_pose):
        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z)) 

        
        
        #cam axi, tool need to move 
        move_cam_x = l.x
        move_cam_y = l.y - cam2tool_y
        move_cam_z = l.z - cam2tool_z
        
        rospy.loginfo('move linear  s(cam_x)='+str(move_cam_x) + ',n(cam_y)='+str(move_cam_y) + ', a(cam_z)='+str(move_cam_z)

        self.Arm.relative_move_nsa(n= move_cam_y, s = move_cam_x, a = move_cam_z -0.05)

        while self.Arm.busy:
            rospy.sleep(.1)


        #----------- Request object pose--------#
        task.Arm.relative_move_nsa(a = 0.03)


if __name__ == '__main__':

    rospy.init_node('t2o_obj_center', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    rospy.loginfo('T2O Ready')


    #----------- Go Photo Pose--------#
    task.robot_photo_pose()
    
    while task.Arm.busy:
        rospy.sleep(.1)
    rospy.loginfo("robot_photo_pose ready!")

    #----------- Request object pose--------#
    #task.obj_pose_request('robots_dvd')

    

    # -------Back 2 home------#.
    #task.safe_pose()
    # task.Arm.home()




    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print 'ok...'
        r.sleep()
