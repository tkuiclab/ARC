#!/usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos

import rospy
import tf

from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse

_POS = (.4, 0, .3)  # x, y, z
_ORI = (-90, 0, 0)  # pitch, roll, yaw


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        #rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False

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

    def relative_control(self, mode='ptp', x=0, y=0, z=0, pitch=0, roll=0, yaw=0):
        """Get euler angle and run task."""
        while self.__is_busy:
            rospy.sleep(.1)

        #fb = task.get_fb()
        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        print 'ori_pos = ' + str(pos)
        print 'ori_euler = ' + str(euler)
        print 'i_pitch = ' + str(pitch)

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
        #rospy.sleep(2.0)

if __name__ == '__main__':

    rospy.init_node('robot_arm_task', anonymous=True)
    rospy.loginfo('robot arm task running')


    task = ArmTask()
    rospy.sleep(1)
    task.pub_ikCmd('ptp')
    rospy.loginfo('Run relative')

    return 

    yaw = 0.504702
    pitch = -3.24753
    roll = 17.6629
    Point = [-0.0273812,0.00313406,0.232947]

    Point[2] = (-1) * Point[2]

    task.relative_control(z=Point[2], pitch = roll )
 
