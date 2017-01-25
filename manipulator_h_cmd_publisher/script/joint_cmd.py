#!/usr/bin/env python

import rospy
import math
import random

# import msgs
from std_msgs.msg import String
from manipulator_h_base_module_msgs.msg import JointPose
from robotis_controller_msgs.msg import StatusMsg

class JointCmd(object):
    def __init__(self):
        self.flg_ready = False
        rospy.init_node('joints_cmd', anonymous=True)

        self.pub_setMode = rospy.Publisher('/robotis/base/set_mode_msg', String, queue_size=0, latch=True)
        self.pub_iniPose = rospy.Publisher('/robotis/base/ini_pose_msg', String, queue_size=0, latch=True)
        self.pub_cmd = rospy.Publisher('/robotis/base/joint_pose_msg', JointPose, queue_size=0, latch=True)

        rospy.Subscriber('robotis/status', StatusMsg, self.msgCallback)
        #rospy.on_shutdown(self.shutdown)

	#def shutdown(self):
		#self.pub_motion(0, 0, 0)
        #rospy.loginfo("Strategy Exit & Stop Robot")

    def run(self):
        self.set_mode()
        self.initial_pose()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if True == self.flg_ready:
                self.pub_cmd.publish( self.generate_cmd() )
                self.flg_ready = False
            #rospy.spin()
            rate.sleep()

    def set_mode(self):
        self.pub_setMode.publish('set_mode')
        rospy.loginfo('Set base module')

    def initial_pose(self):
        self.pub_iniPose.publish('ini_pose')
        rospy.loginfo('Go initial pose')

    def generate_cmd(self):
        cmd = JointPose()
        for i in range(1, 7):
            cmd.name.append ('joint' + str(i))
            if 3 == i:
                angle = math.radians(random.uniform(-90, 0))    # random -90.0~0.0, and convert to rad
            else:
                angle = math.radians(random.uniform(-45, 45))
            cmd.value.append(angle)
        return cmd

    def msgCallback(self, msg):
        rospy.loginfo(self.logLevel(msg.type) + msg.module_name + ' ' + msg.status_msg)
        if 'End Trajectory' == msg.status_msg:
            self.flg_ready = True

    def logLevel(self, type):
        if   1 == type:
            return '[INFO] '
        elif 2 == type:
            return '[WARN] '
        elif 3 == type:
            return '[ERROR] '
        return '[UNKNOW] '

if __name__ == '__main__':
    try:
        cmd = JointCmd()
        cmd.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Pushiler is closed')
