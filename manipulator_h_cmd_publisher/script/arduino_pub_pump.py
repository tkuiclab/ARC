#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool    

class com_arduino(object):
    def __init__(self):
        self.__pub = rospy.Publisher('pump', Bool, queue_size=10)
    
    def __del__(self):
        self.__pub.publish(False)

    def pump_ctrl(self, sw):
        if not rospy.is_shutdown():
            self.__pub.publish(sw)

    def pump_test(self):
        rospy.loginfo('communicate arduino')
        rate = rospy.Rate(2)

        out = True
        while not rospy.is_shutdown():
            self.__pub.publish(out)
            out = not out
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('com_arduino', anonymous=True)
    com = com_arduino()
    com.pump_test()
