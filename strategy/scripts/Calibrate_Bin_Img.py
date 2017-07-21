#!/usr/bin/env python
"""description"""

# pylint: disable = invalid-name
# pylint: disable = C0326
# pylint: disable = W0105, C0303

# import roslib
# import rospkg
# import rospy
# from std_msgs.msg import Bool, Char, Float64, String

# import arm_task_rel
# import LM_Control

# from config import *

#
# import math
# import threading
# import time
# import numpy

# import actionlib
# import roslaunch
# import roslib;roslib.load_manifest('obj_pose')
# import rospkg
import rospy

# import obj_pose.msg
# from actionlib_msgs.msg import GoalID, GoalStatusArray
# from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String
# from strategy.srv import *

# import arm_task_rel
import LM_Control 
# from task_parser import *
from config import *
# from gripper import *
#
BinArr = ['a',  'b',  'c',  'd',  'e',  'f',  'g',  'h',  'i',  'j']
# BinArr = ['a',  'b',  'c',  'd',  'e']
class Calibrater:
    def __init__(self):
        self.LM  = LM_Control.CLM_Control()
        # self.Arm = arm_task_rel.ArmTask()
        # rospy.on_shutdown(self.shutdown)
    
    def Move_LM(self, bin):
        # print 'self.LM.IsBusy = ' + str(self.LM.IsArrive)
        while self.LM.IsArrive == False:
            # print 'wait'
            rospy.sleep(0.01)
        
        print 'move 2 bin ->' + bin
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin ) + LM_Right_Arm_Shift)
        rospy.sleep(0.5)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin ))
        rospy.sleep(0.5)
        # while self.LM.IsArrive == True:
        #     print 'wait2'
        #     rospy.sleep(0.5)

    
    def Show_LM_IsBusy(self):
        print 'LM IS busy == '+ str(self.LM.IsBusy)


if __name__ == '__main__':

    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('Calibrate_Bin_Img', anonymous=True)
        calibrate = Calibrater()

        # for single cmd
        # calibrate.Move_Arm()
        calibrate.Move_LM('c')
        exit()
        # for continue caltbration task
        r = rospy.Rate(30)
        i = 0
        while not rospy.is_shutdown() and i < len(BinArr):
            calibrate.Move_LM(BinArr[i])
            i = i+1
        # ==================================================

        r.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
