#!/usr/bin/env python

import rospy
import arm_task_rel


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
        self.Arm = arm_task_rel.ArmTask()
        # rospy.on_shutdown(self.shutdown)
    
    def Move_LM(self, bin):
        # print 'self.LM.IsBusy = ' + str(self.LM.IsArrive)
        while self.LM.IsArrive == False:
            # print 'wait'
            rospy.sleep(0.01)
        
        print 'move 2 bin ->' + bin
        self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', bin ) + LM_Right_Arm_Shift)
        rospy.sleep(0.5)
        self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', bin ) - 2000)
        rospy.sleep(0.5)
        # while self.LM.IsArrive == True:
        #     print 'wait2'
        #     rospy.sleep(0.5)

    
    def Show_LM_IsBusy(self):
        print 'LM IS busy == '+ str(self.LM.IsBusy)

    def safe_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)


    def arm_2_bin_front(self):
        self.Arm.pub_ikCmd('ptp', (0.2, 0.0 , 0.4), (-100, 0, 0) )
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)

    
if __name__ == '__main__':

    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('Calibrate_Bin_Img', anonymous=True)
        calibrate = Calibrater()
        #calibrate.safe_pose()
        calibrate.arm_2_bin_front()
        # for single cmd
        # calibrate.Move_LM('e')
        # exit()
        # for continue caltbration task
        r = rospy.Rate(1)
        # i = 0
        i = 0

        calibrate.Move_LM(BinArr[i])
        # while not rospy.is_shutdown() and i < len(BinArr):
            # calibrate.Move_LM(BinArr[i])
            # i = i+1
        # ==================================================

        # r.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
