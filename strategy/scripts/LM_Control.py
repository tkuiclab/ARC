#!/usr/bin/env python
"""description"""

# pylint: disable = invalid-name
# pylint: disable = C0326
# pylint: disable = W0105, C0303


import sys
import rospy
import arm_task_rel

from std_msgs.msg import Int32
from std_msgs.msg import String
from linear_motion.msg   import LM_Cmd
from vacuum_cmd_msg.srv import VacuumCmd

TargetId =      ['a',  'b',  'c',  'd',  'e',  'f',  'g',  'h',  'i',  'j',   'k',   'l']
TargetShift_X = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 50000, 65000, 80000]
TargetShift_Z = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 40000, 60000]

class CLM_Control:
    def __init__(self):
        self.__is_busy = False
        self.__is_Arrive = False
        self.__set_pubSub()
        self.x_curr_pos=0
        self.z_curr_pos=0

    def Show_FB_callback(self, msg):
        """ description """
        # print msg.status
        self.z_curr_pos = msg.x_curr_pos
        self.x_curr_pos = msg.z_curr_pos
        if msg.status == 'LM_idle':
            self.__is_busy = False
            self.__is_Arrive = False

        elif msg.status == 'LM_busy':
            # print 'LM_busy'  
            self.__is_busy = True
            self.__is_Arrive = False
                
        elif msg.status == 'LM_complete':
            # print 'LM_complete'         # execute arm task
            self.__is_busy = False
            self.__is_Arrive = True
        else:
            print 'err'

    def __set_pubSub(self):
        self.set_pls_pub = rospy.Publisher(
            '/position_topic',
            LM_Cmd,
            latch = True,
            queue_size=10
        )
        self.Get_FB_sub = rospy.Subscriber(
            '/LM_FeedBack', 
            LM_Cmd, 
            self.Show_FB_callback,
            queue_size=10
        )

    
    @property
    def IsBusy(self):
        return self.__is_busy

    @property
    def IsArrive(self):
        return self.__is_Arrive

    def Show_LM_FB(self):
        print 'x_curr_pos = ' + str(self.x_curr_pos)
        print 'z_curr_pos = ' + str(self.z_curr_pos)

    def rel_move_LM(self, LM_Name, cm):
        tmp_x = self.x_curr_pos
        tmp_z = self.z_curr_pos
        add = 1000*cm

        if LM_Name == 'left':  #id=1
            self.pub_LM_Cmd(1, tmp_z - add)
            rospy.sleep(0.3)
            
        elif LM_Name == 'base':#id=2
            self.pub_LM_Cmd(2, tmp_x - add)
            
        else:
            print'err, please enter "left", "right" or "base" '
        while(self.__is_Arrive == False):
            rospy.sleep(0.1)
            # print'wait'

    def pub_LM_Cmd(self, id, pls):
        msg = LM_Cmd()
        msg.id = id
        # print 'x_curr_pos = ' + str(self.x_curr_pos)
        # print 'z_curr_pos = ' + str(self.z_curr_pos)
        # print 'LM ' + str(id) + 'send pls = ' + str(pls)
        if id == 1:                                             #right
            if pls > 80000 or pls < 0:
                print 'error pls for Z-dir LM (0 ~ 80000)'
                return
            else:
                msg.x = pls

        elif id == 2:                                             #base
            if pls > 60000 or pls < 0:
                print 'error pls for Z-dir LM (0 ~ 80000)'
                return 
            else:
                msg.z = pls

        elif id == 3:                                             #left
            if pls > 80000 or pls < 0:
                print 'error pls for left LM (0 ~ 80000)'
                return 
            else:
                msg.left = pls
            print 'sss'

        elif id == 4:
            msg.z = pls
            msg.x = pls
        elif id == 5:
            msg.z = pls
            msg.x = pls
        else:
            print 'error input arg for pub_LM_Cmd(self, id, pls)'
        self.set_pls_pub.publish(msg)
        


# def GetShift(LM_Dir):
#     """ description """
#     if ch.data in TargetId:
#         if LM_Dir == 'x':
#             return TargetShift_X[TargetId.index(ch.data)]
#         elif LM_Dir=='z':
#             return TargetShift_Z[TargetId.index(ch.data)]
#         else:
#             print 'Error input dir'
#     else:
#         print 'Error input character'

if __name__ == '__main__':
    """ Check and initialize input parameters """
    if len(sys.argv) == 3:
        id =    int(sys.argv[1])
        pls = int(sys.argv[2])
        print 'OK'
    else:
        print 'error input'
        id  = 1
        pls = 0

    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('LinearMove', anonymous=True)
        #=======================
        LM = CLM_Control()
        LM.pub_LM_Cmd(id, pls) # Control linear motor
        # LM.Vaccum_Test(True)
        # rospy.sleep(1)
        # LM.Vaccum_Test(False)
        rospy.loginfo('running')
        rate = rospy.Rate(10) # 10hz
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
