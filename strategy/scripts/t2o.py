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

#[0.3, 0, 0.3, -60, 0, 0, 0];
_POS = (.2, 0, .3)  # x, y, z
_ORI = (-70, 0, 0)  # pitch, roll, yaw


#cam2tool_y = -0.04  #cam axis
#cam2tool_z = 0.195 + 0.035

cam2tool_y = -0.095  #cam axis
cam2tool_z = 0.25 + 0.035


class T2O:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        #self.__set_pubSub()
        self.Arm 			= arm_task_rel.ArmTask()

        rospy.on_shutdown(self.stop_task)
        self.Arm.busy = False
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        
        
    def stop_task(self):
        """Stop task running."""
        self.Arm.stop_task()
        
    #request object pose
    def obj_pose_request(self, obj):
        while self.Arm.busy:
            rospy.sleep(.1)
        rospy.loginfo('obj_pose_request()')

        #goal = obj_pose.msg.ObjectPoseGoal("expoEraser")
        #goal = obj_pose.msg.ObjectPoseGoal("irishSpring")
        
        goal = obj_pose.msg.ObjectPoseGoal(obj)

        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

    def desk_photo_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.30, 0.0 , 0.2), (-180, 0, 0) )

    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

        
    def obj_pose_done_cb(self, state, result):
        self.arm_2_obj(result.object_pose)

    def arm_2_obj(self, obj_pose):
        p = object_pose
        a = p.angular
        l = p.linear

        #rospy.loginfo("object_pose = " + str(p))
        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z)) 
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z))  ) 
        
                        
        pitch = numpy.rad2deg(a.x ) 
        
        pitch = (pitch - 180) if pitch > 90  else pitch
        pitch = (pitch + 180) if pitch < -90  else pitch


        pitch = pitch *(-1)
        
        rospy.loginfo('vision give -pitch=' + str(pitch))
        
        
        
        rospy.loginfo('move rotation pitch=' + str(pitch))

        #return
        self.relative_control_rotate( pitch = pitch)
   
        
        while self.Arm.busy:
            rospy.sleep(.1)

        
        #return
        
        #cam axi, tool need to move 
        move_cam_x = l.x
        move_cam_y = l.y - cam2tool_y
        move_cam_z = l.z - cam2tool_z
        

        rospy.loginfo('move linear n(cam_y)='+str(move_cam_y) + ', s(cam_x)='+str(move_cam_x)  + ', a(cam_z)='+str(move_cam_z))
        #self.relative_xyz_base(x = )
        
        rospy.loginfo('move linear base_x='+str(-move_cam_y) +
             ', base_y='+str(move_cam_x)  +
              ', base_z='+str(-move_cam_z))
        

        self.Arm.elative_move_nsa(n = move_cam_y , s= -move_cam_x, a = move_cam_z)


if __name__ == '__main__':

    rospy.init_node('t2o', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    # task.desk_photo_pose()

    #self.Arm.pub_ikCmd('ptp', (0.30, 0.0 , 0.2), (-180, 0, 0) )

    #task.Arm.pub_ikCmd('ptp', (0.40, 0.0 , 0.2), (-180, 0, 0) )

    # task.Arm.relative_move_nsa(n =  0.05) # cam_y
    # task.Arm.relative_move_nsa(s = -0.05) # cam_x
    task.Arm.relative_move_nsa(a = -0.05) # cam_z

    # task.Arm.relative_rot_nsa(s = 30)     # pitch -> cam_x
    # task.Arm.relative_rot_nsa(a = 10)     # cam_z
    # task.Arm.relative_rot_nsa(n = 10)     # cam_y

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print 'ok...'
        r.sleep()
