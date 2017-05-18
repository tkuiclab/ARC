#!/usr/bin/env python

"""Use to generate arm task and run."""

import rospy

from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd
from std_srvs.srv import SetBool

import sys
from math import radians, degrees

_PITCH_MAX, _PITCH_MIN =   0, -15
_YAW_MAX,   _YAW_MIN   =  20, -20

_STEP_P = 5
_STEP_Y = 10

_POS = (0, 0.33, 0.42) # x, y, z
_ORI = (0, -3, 0)  # pitch, roll, yaw

_WRIST_LEN = 0.33 - 0.16


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__generator = self.gen_nextEuler()
        self.__is_busy = False
        self.__dis_m = None
        self.__flag = False

    def __set_pubSub(self):
        self.__set_mode_pub = rospy.Publisher(
            '/robotis/base/set_mode_msg',
            String,
            latch=True,
            queue_size=1)

        self.__set_endlink_pub = rospy.Publisher(
            '/robotis/base/set_endlink',
            Float64,
            latch=True,
            queue_size=1)

        self.__ptp_pub = rospy.Publisher(
            '/robotis/base/JointP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1)

        self.__cmd_pub = rospy.Publisher(
            '/robotis/base/TaskP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1)

        self.__status_sub = rospy.Subscriber(
            '/robotis/status',
            StatusMsg,
            self.__status_callback,
            queue_size=10)

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

        # if change tool length
        if self.__dis_m is not None:
            # pos: y = y + dis - _WRIST_LEN
            cmd[1] = cmd[1] + self.__dis_m - _WRIST_LEN

        if 'line' == mode:
            self.__cmd_pub.publish(cmd)
        elif 'ptp' == mode:
            self.__ptp_pub.publish(cmd)

        self.__is_busy = True

    def stop_task(self):
        """Stop task running."""
        self.__set_mode_pub.publish('')

    def set_endlink(self, dis_m):
        self.__dis_m = dis_m
        self.__set_endlink_pub.publish(dis_m)

    def gen_nextEuler(self):
        """Generator euler angle."""
        p, y = _PITCH_MAX, _YAW_MAX

        for p in range(_PITCH_MAX, _PITCH_MIN-_STEP_P, -_STEP_P):
            if p % 2 == 0: 
                for y in range(_YAW_MAX, _YAW_MIN-_STEP_Y, -_STEP_Y):
                    yield (p, y)
            else:
                for y in range(_YAW_MIN, _YAW_MAX+_STEP_Y, _STEP_Y):
                    yield (p, y)

    def run(self):
        """Get euler angle and run task."""
        if self.__is_busy:
            return
        else:
            if self.__flag:
                rospy.sleep(.5)
                req = rospy.ServiceProxy('/save_img', SetBool)
                res = req(True)
            else:
                self.__flag = True

            try:
                p, y = self.__generator.next()
                self.pub_ikCmd('ptp', euler=(p, 0, y))
            except StopIteration:
                rospy.loginfo('Taking pictures of this time is done.')
                raw_input('Please press <Enter> key to continue.')
                self.__generator = self.gen_nextEuler()
                self.__flag = False
                req = rospy.ServiceProxy('/save_img', SetBool)
                res = req(False)


if __name__ == '__main__':

    rospy.init_node('strategy', anonymous=True)
    rospy.loginfo('Strategy Running')
    dis_m = rospy.get_param('distance', 0.6)
    velocity = rospy.get_param('velocity', 60)

    return

    set_vel_pub = rospy.Publisher(
        '/robotis/base/set_velocity',
        Float64,
        latch=True,
        queue_size=1
    )
    set_vel_pub.publish(velocity)

    task = ArmTask()
    task.set_endlink(dis_m)
    rospy.sleep(1)
    task.pub_ikCmd('ptp')

    try:
        rospy.wait_for_service('/save_img')

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            task.run()
            rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo('ROS is closed.')
    except Exception as e:
        print e
