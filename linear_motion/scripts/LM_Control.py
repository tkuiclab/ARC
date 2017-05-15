#!/usr/bin/env python
"""description"""

# pylint: disable = invalid-name
# pylint: disable = C0326
# pylint: disable = W0105

import sys
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import String
from linear_motion.msg   import LM_Cmd

TargetId =      ['a',  'b',  'c',  'd',  'e',  'f',  'g',  'h',  'i',  'j',   'k',   'l']
TargetShift_X = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 40000, 60000, 80000]
TargetShift_Z = [  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 40000, 60000]

def GetShift(LM_Dir):
    """ description """
    if ch.data in TargetId:
        if LM_Dir == 'x':
            return TargetShift_X[TargetId.index(ch.data)]
        elif LM_Dir=='z':
            return TargetShift_Z[TargetId.index(ch.data)]
        else:
            print 'Error input dir'
            return 0
    else:
        print 'Error input character'

if __name__ == '__main__':
    """ Check and initialize input parameters """
    if len(sys.argv) == 3:
        id =    int(sys.argv[1])
        ch = String(sys.argv[2])
    else:
        print 'error input arguments'
        sys.exit(1)

    """ Initialize ros node and publish cmd """
    try:
        rospy.init_node('LinearMove', anonymous=True)
        rospy.loginfo('running')

        set_pls_pub = rospy.Publisher(
            '/position_topic',
            LM_Cmd,
            latch = True,
            queue_size=1
        )
        msg = LM_Cmd()
        msg.id = id
        msg.x  = GetShift('x')
        msg.z  = GetShift('z')
        print msg.x
        print msg.z

        set_pls_pub.publish(msg)
        rate = rospy.Rate(10) # 10hz
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
