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
from gripper import cam2tool_y, cam2tool_z
import s

obj_dis = 0.1

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
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")") 
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" ) 
        
                        
        r = numpy.rad2deg(a.x)
        p = numpy.rad2deg(a.y)
        

        move_cam_x = l.x
        move_cam_y = l.y - cam2tool_y
        move_cam_z = l.z - cam2tool_z 
        
        #return
        #----------------Rotation---------------_#
        #self.Arm.relative_rot_nsa(pitch = r)  #roll
        #self.Arm.relative_rot_nsa(yaw = p)  #pitch
        #self.Arm.relative_rot_nsa(pitch = r, yaw = p)  #pitch
        self.Arm.relative_rot_pry_move_nsa(pitch = r, yaw = p, n= move_cam_y, s = move_cam_x, a = move_cam_z -obj_dis)
        

        while self.Arm.busy:
            rospy.sleep(.1)

        rospy.loginfo('Move Angle Finish')

        
        #return
        
        #----------------Move---------------_#

        rospy.loginfo('move linear  s(cam_x)='+str(move_cam_x) + ',n(cam_y)='+str(move_cam_y) + ', a(cam_z)='+str(move_cam_z -obj_dis) )

        #self.Arm.relative_move_nsa(n= move_cam_y, s = move_cam_x, a = move_cam_z -obj_dis)
        

        #self.Arm.relative_move_nsa(n= l.y, s = l.x)
        #self.Arm.relative_move_nsa(n= l.y, s = l.x, a = l.z -obj_dis)

        
        #----------------Rotation+Move---------------_#
        #rospy.loginfo('move linear  s(cam_x)='+str(move_cam_x) + ',n(cam_y)='+str(move_cam_y) + ', a(cam_z)='+str(move_cam_z) )
        #self.Arm.relative_move_nsa_rot_pry(pitch = -r, yaw = -p, s = move_cam_x, n = move_cam_y, a = move_cam_z)

        
        #---------------Suction---------------#
        while self.Arm.busy:
            rospy.sleep(.1)
        
        task.Arm.relative_move_nsa( a = obj_dis -0.01)



if __name__ == '__main__':

    rospy.init_node('t2o_robot', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    rospy.loginfo('T2O Ready')

    # task.safe_pose()
   # task.robot_photo_pose()
    

    # Problem
    # task.Arm.relative_rot_nsa(pitch = 2.125044, yaw = 7.428986)
    # task.Arm.relative_move_nsa(n= 0.197568, s = -0.053241, a = 0.309954 -obj_dis)
    # ->
    # task.Arm.relative_rot_pry_move_nsa(pitch = 2.125044, yaw = 7.428986, n= 0.197568, s = -0.053241, a = 0.309954 -obj_dis)
    # task.Arm.relative_move_nsa( a = obj_dis)

    # task.robot_photo_pose()
    # task.Arm.relative_rot_nsa(pitch = 9.328644, yaw = 35.861837)
    # task.Arm.relative_move_nsa(n= 0.306312, s = -0.110500, a = 0.254350 -obj_dis)

    #task.Arm.relative_move_nsa( a = 0.1)
    # exit()

    #----------- Go Photo Pose--------#
    task.robot_photo_pose()
    
    # while task.Arm.busy:
    #     rospy.sleep(.1)
    

    #----------- Request object pose--------#
    # task.obj_pose_request('robots_dvd')

    

    # -------Back 2 home------#.
    # task.safe_pose()
    # task.Arm.home()


    # -------Relative Test------#
    # task.Arm.relative_rot_nsa(pitch = -14.7) 
    # while task.Arm.busy:
    #     rospy.sleep(.1)
    # task.Arm.relative_move_nsa(s = 0.0167663395405 ,n = 0.0226672434807, a=0.262694020271-0.05)
    
    # task.Arm.relative_move_nsa_rot_pry(pitch = -14.7,s = 0.0167663395405 ,n = 0.0226672434807, a=0.262694020271-0.05)

    #task.Arm.relative_move_nsa(n =  0.02) # cam_y
    #task.Arm.relative_move_nsa(s = 0.05) # cam_x
    #task.Arm.relative_move_nsa(a =  0.01) # cam_z


    # task.Arm.relative_rot_nsa(pitch =  10)     # pitch -> cam_x
    task.Arm.relative_rot_nsa(roll =  -200)     # cam_z
    # task.Arm.relative_rot_nsa(yaw = -10)     # cam_y

    #---------IK FAIL-----------$
    # task.robot_photo_pose()
    # task.Arm.relative_rot_nsa(pitch = -34.457731, yaw = 1.510902)
    
 

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print 'ok...'
        r.sleep()
