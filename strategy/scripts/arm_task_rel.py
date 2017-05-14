#! /usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos
from numpy import multiply

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



_POS = (.3, 0, .15)  # x, y, z
_ORI = (-40, 0, 0)  # pitch, roll, yaw


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        rospy.loginfo('Wait /ObjEst action...')
        self.__obj_pose_client.wait_for_server()

    def __set_pubSub(self):
        self.__set_mode_pub = rospy.Publisher(
            '/robotis/base/set_mode_msg',
            String,
            latch=True,
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
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.stop_task()

        elif 'End Trajectory' in msg.status_msg:
            self.__is_busy = False

    def pub_ikCmd(self, mode='line', pos=_POS, euler=_ORI):
        """Publish ik cmd msg to manager node."""
        cmd = []

        for p in pos:
            cmd.append(p)
        for e in euler:
            cmd.append(e)

        rospy.loginfo('Sent:{}'.format(cmd))

        if 'line' == mode:
            self.__cmd_pub.publish(cmd)
        elif 'ptp' == mode:
            self.__ptp_pub.publish(cmd)

        self.__is_busy = True

    def stop_task(self):
        """Stop task running."""
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

    def relative_control(self, mode='ptp', n=0, s=0, a=0):
        """Get euler angle and run task."""
        while self.__is_busy:
            rospy.sleep(.1)

        fb = task.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        rot = self.euler2rotation(euler)
        vec_n, vec_s, vec_a = self.rotation2vector(rot)

        if n != 0:
            move = multiply(vec_n, n)
        elif s != 0:
            move = multiply(vec_s, s)
        elif a != 0:
            move = multiply(vec_a, a)
        else:
            move = [0, 0, 0]

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
    


    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

        
    def obj_pose_done_cb(self, state, result):
        #rospy.loginfo("In obj_pose_done_cb")
        rospy.loginfo("object_pose = " + str(result))
        twist = result.object_pose

    #request object pose
    def obj_pose_request(self):
        rospy.loginfo('obj_pose_request()')

        goal = obj_pose.msg.ObjectPoseGoal("Windex")
        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

if __name__ == '__main__':

    rospy.init_node('robot_arm_task', anonymous=True)
    rospy.loginfo('robot arm task running')

    rospy.sleep(0.5)

    rospy.loginfo('after sleep 0.,5')


    task = ArmTask()
    task.obj_pose_request()

    # task.pub_ikCmd('ptp')
    # task.relative_control(s=.1)

#    case ORDER_ZYX:
#         Mx.M[0][0]=Cy*Cz;
#         Mx.M[0][1]=Cz*Sx*Sy-Cx*Sz;
#         Mx.M[0][2]=Cx*Cz*Sy+Sx*Sz;
#         Mx.M[1][0]=Cy*Sz;
#         Mx.M[1][1]=Cx*Cz+Sx*Sy*Sz;
#         Mx.M[1][2]=-Cz*Sx+Cx*Sy*Sz;
#         Mx.M[2][0]=-Sy;
#         Mx.M[2][1]=Cy*Sx;
#         Mx.M[2][2]=Cx*Cy;
#         break;
