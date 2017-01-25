#!/usr/bin/env python

''' This is pick and placerun demo script '''

import math
import random
import rospy

# import msgs
from sensor_msgs.msg import JointState

def generate_cmd(new=True):
    cmd = JointState()
    cmd.header.stamp = rospy.get_rostime()

    for i in range(1, 7):
        cmd.name.append('joint' + str(i))
        if 3 == i:
            angle = math.radians(random.uniform(-90, 0))    # random -90.0~0.0, and convert to rad
        else:
            angle = math.radians(random.uniform(-45, 45))
        cmd.position.append(angle)
    return cmd

def run_random():
    rate = rospy.Rate(2)
    while not rospy.is_shutdown(): 
        pub_js.publish(generate_cmd())
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_pub', anonymous=True)
        pub_js = rospy.Publisher('robotis/pro7a/goal_joint_states', JointState, queue_size=10, latch=True)

        rospy.loginfo('start...')
        run_random()

    except Exception as e:
        print 'except: ', e
