#! /usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos, pi
from numpy import multiply
import numpy

import rospy
import roslib;roslib.load_manifest('obj_pose')
import tf


import actionlib
from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse

import obj_pose.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import json


#[0.3, 0, 0.3, -60, 0, 0, 0];
_POS = (.2, 0, .3)  # x, y, z
_ORI = (-70, 0, 0)  # pitch, roll, yaw


#cam2tool_y = -0.04  #cam axis
#cam2tool_z = 0.195 + 0.035

cam2tool_y = -0.095  #cam axis
cam2tool_z = 0.25 + 0.035


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        rospy.on_shutdown(self.stop_task)
        #self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        #rospy.loginfo('Wait /obj_pose action...')
        #self.__obj_pose_client.wait_for_server()

    def __set_pubSub(self):
        self.__set_mode_pub = rospy.Publisher(
            '/robotis/base/set_mode_msg',
            String,
            latch=False,
            queue_size=1
        )

        self.__ptp_pub = rospy.Publisher(
            '/robotis/base/JointP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1
        )

        self.__cmd_pub = rospy.Publisher(
            '/robotis/base/TaskP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1
        )

        self.__status_sub = rospy.Subscriber(
            '/robotis/status',
            StatusMsg,
            self.__status_callback,
            queue_size=10
        )

        

    def __status_callback(self, msg):
        rospy.loginfo('msg.status_msg= '+ msg.status_msg)
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.stop_task()

        elif 'End Trajectory' in msg.status_msg:
            rospy.loginfo('In End Trajectory')
            self.__is_busy = False

    def pub_ikCmd(self, mode='line', pos=_POS, euler=_ORI):
        """Publish ik cmd msg to manager node."""
        
        cmd = []

        for p in pos:
            cmd.append(p)
        for e in euler:
            cmd.append(e)

        rospy.loginfo('Sent:{}'.format(cmd))
        self.__is_busy = True  #!!!!!!!!!

        if 'line' == mode:
            self.__cmd_pub.publish(cmd)
        elif 'ptp' == mode:
            self.__ptp_pub.publish(cmd)

    def set_mode(self):    
        self.__set_mode_pub.publish('set')

    def stop_task(self):
        """Stop task running."""
        self.__is_busy = False
        self.__set_mode_pub.publish('')

    def get_fb(self):
        rospy.wait_for_service('/robotis/base/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                '/robotis/base/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def quaternion2euler(self, ori):
        quaternion = (
            ori.x,
            ori.y,
            ori.z,
            ori.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        # if degrees( abs(roll) )  > 90 and  degrees( abs(yaw) )  > 90: 
        #     pitch = -pi/2 -(pi/2 - abs(pitch) )
        #     roll = 0
        #     yaw = 0

        return (pitch, roll, yaw)

    def euler2rotation(self, euler):
        Cx = cos(euler[0])
        Sx = sin(euler[0])
        Cy = cos(euler[1])
        Sy = sin(euler[1])
        Cz = cos(euler[2])
        Sz = sin(euler[2])

        return [
            [Cz * Sy + Sz * Sx * Cy,  Cz * Cy - Sz * Sx * Sy, -Sz * Cx],
            [Sz * Sy - Cz * Sx * Cy,  Sz * Cy + Cz * Sx * Sy,  Cz * Cx],
            [Cx * Cy, -Cx * Sy,  Sx]
        ]

    def rotation2vector(self, rot):
        vec_n = [rot[0][0], rot[1][0], rot[2][0]]
        vec_s = [rot[0][1], rot[1][1], rot[2][1]]
        vec_a = [rot[0][2], rot[1][2], rot[2][2]]
        return vec_n, vec_s, vec_a

    def relative_control_rotate(self, mode='ptp', x=0, y=0, z=0, pitch=0, roll=0, yaw=0):
        """Get euler angle and run task."""
        while self.__is_busy:
            rospy.loginfo('busy..')
            rospy.sleep(.1)

        #fb = task.get_fb()
        fb = self.get_fb()
        

        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

    

        #print 'ori_pos = ' + str(pos)
        #print 'ori_euler = ' + str(euler)
        #print 'i_pitch = ' + str(pitch)

        self.pub_ikCmd(
            mode,
            (pos.x + x, pos.y + y, pos.z + z),
            (
                degrees(euler[0]) + pitch,
                degrees(euler[1]) + roll,
                degrees(euler[2]) + yaw
            )
        )

        while self.__is_busy:
           rospy.sleep(.1)

    def relative_control(self, mode='ptp', n=0, s=0, a=0):
        """Get euler angle and run task."""
        while self.__is_busy:
            rospy.sleep(.1)

        #fb = task.get_fb()
        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        rot = self.euler2rotation(euler)
        vec_n, vec_s, vec_a = self.rotation2vector(rot)

        move = [0, 0, 0]
        if n != 0:
            move += multiply(vec_n, n)
        if s != 0:
            move += multiply(vec_s, s)
        if a != 0:
            move += multiply(vec_a, a)

        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
            (
                degrees(euler[0]),
                degrees(euler[1]),
                degrees(euler[2])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)
    
    def relative_xyz_base(self, mode='ptp', x=0, y=0, z=0):
        """Get euler angle and run task."""
        while self.__is_busy:
            rospy.sleep(.1)

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        self.pub_ikCmd(
            mode,
            (pos.x + x, pos.y + y, pos.z + z),
            (
                degrees(euler[0]),
                degrees(euler[1]),
                degrees(euler[2])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

        
    def obj_pose_done_cb(self, state, result):
        p = result.object_pose
        l = p.linear

        #rospy.loginfo("object_pose = " + str(p))
        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(p.linear.x) + ", " + str(p.linear.y)+ ", " + str(p.linear.z)) 
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(p.angular.x)) + ", " 
                        + str(numpy.rad2deg(p.angular.y)) + ", " 
                        + str(numpy.rad2deg(p.angular.z))  ) 
        
                        
        pitch = numpy.rad2deg(p.angular.x ) 
        
        pitch = (pitch - 180) if pitch > 90  else pitch
        pitch = (pitch + 180) if pitch < -90  else pitch


        pitch = pitch *(-1)
        
        rospy.loginfo('vision give -pitch=' + str(pitch))
        
        
        
        rospy.loginfo('move rotation pitch=' + str(pitch))

        #return
        #self.relative_control_rotate( pitch = pitch)
        #task.pub_ikCmd('ptp', (0.40, 0.05 , 0.15), (-90 +pitch, 0, 0) )
        
        
        while self.__is_busy:
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
        

        #self.relative_control(n = move_cam_y , s= -move_cam_x, a = move_cam_z)
        self.relative_xyz_base(x = -move_cam_y, y = move_cam_x,z = -move_cam_z)
    

    #request object pose
    def obj_pose_request(self):
        while self.__is_busy:
            rospy.sleep(.1)
        rospy.loginfo('obj_pose_request()')

        #goal = obj_pose.msg.ObjectPoseGoal("expoEraser")
        #goal = obj_pose.msg.ObjectPoseGoal("irishSpring")
        
        goal = obj_pose.msg.ObjectPoseGoal("dvdRobots")

        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

if __name__ == '__main__':

    rospy.init_node('robot_arm_task', anonymous=True)


    task = ArmTask()
    rospy.sleep(0.5)
    task.set_mode()
    rospy.sleep(0.2)

    #task.pub_ikCmd('ptp')
    

    #init pose
    #task.pub_ikCmd('ptp', (0.30, 0.0 , 0.22), (0, 0, 0) )
    task.pub_ikCmd('ptp', (0.20, 0.0 , 0.3), (-70, 0, 0) )
		
    #stow photo pose 
    #task.pub_ikCmd('ptp', (0.40, 0.00 , 0.15), (-90, 0, 0) )
    
    #task.relative_xyz_base(y = 0.1)
    
    rospy.loginfo('strategy ready')
    #task.obj_pose_request()

    #task.relative_control(n=.05)  # -cam_y
    #task.relative_control(s=.05)  # cam_x
    task.relative_control(a=.05)  #cam_z

    #task.relative_control_rotate( pitch = -5 )
    #task.relative_control_rotate( pitch = -5 )
    
    #move 13 cm can get object
    #tool 9.5
    #want only base_x(cam)=-4cm

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print 'ok...'
        r.sleep()

    #want height 24cm, but 28cm
    #task.relative_control(s=.05)  #-y
    #task.relative_control(a=.05)   # cam_z
    #task.relative_control_rotate( pitch = roll )
 
#    case ORDER_ZYX:
#         Mx.M[0][0]=Cy*Cz;
#         Mx.M[0][1]=Cz*Sx*Sy-Cx*Sz;
#         Mx.M[0][2]=Cx*Cz*Sy+Sx*Sz;
#         Mx.M[1][0]=Cy*Sz;
#         Mx.M[1][1]=Cx*Cz+Sx*Sy*Sz;
#         Mx.M[1][2]=-Cz*Sx+Cx*Sy*Sz;0.5
#         Mx.M[2][0]=-Sy;
#         Mx.M[2][1]=Cy*Sx;
#         Mx.M[2][2]=Cx*Cy;
#         break;
