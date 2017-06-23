#!/usr/bin/env python
"""description"""

# pylint: disable = invalid-name
# pylint: disable = C0326
# pylint: disable = W0105, C0303

import roslib
import rospkg
import rospy
from std_msgs.msg import Bool, Char, Float64, String

import arm_task_rel
import LM_Control

from config import *

class Calibrater:
    def __init__(self):
        self.LM = LM_Control.CLM_Control()
        self.aa = 12
    
    def Move(self, bin):
		print 'test_go_bin_LM bin -> ' + bin
		self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin ))
		rospy.sleep(0.3)
		self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin ))
        # self.Show_LM_IsBusy()
    
    def Show_LM_IsBusy(self):
        print 'LM IS busy == '+ str(calibrate.LM.IsBusy)

        

if __name__ == '__main__':

    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('Calibrate_Bin_Img', anonymous=True)
        #=======================
        rate = rospy.Rate(30)  # 30hz
    
        calibrate = Calibrater()
        calibrate.Move('h')
        while not rospy.is_shutdown():
            calibrate.Show_LM_IsBusy()
            rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass

# ==============================================================
        # rate = rospy.Rate(30)  # 30hz
		# while not rospy.is_shutdown():
		# 	LM.pub_LM_Cmd(id, pls) # Control linear motor
			
		# 	rate.sleep()