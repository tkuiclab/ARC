#!/usr/bin/env python

''' this progam is maker faire demo for marking '''

import sys
import math
import copy
import rospy

from kinematics_cmd import pose_cmd, pose_type

def wait_moving():
    while pcmd.arm_is_moving():
        rospy.Rate(10).sleep()
        if rospy.is_shutdown():
            sys.exit(1)

def send_cmd(cmd):
    if rospy.is_shutdown():		
        sys.exit(1)
    
    wait_moving()
    pcmd.publish_cmd(pcmd.cmd_to_kinematics_msg(cmd))

def pick_place(pick, place, rot=0.0):
    pick_up = copy.deepcopy(pick)
    pick_up.position[2] += 0.1      # z
    send_cmd(pick_up)
    send_cmd(pick)
    send_cmd(pick_up)

    place_up = copy.deepcopy(place)
    place_up.position[2] += 0.1     # z
    send_cmd(place_up)
    send_cmd(place)
    send_cmd(place_up)

def run():
    pick   = pose_type([0.2, -0.32, -0.016], [0.0, 90.0, 0.0])
    place  = pose_type([0.2, -0.25, -0.025], [0.0, 90.0, 0.0])

    pick_place(pick, place)

if __name__ == '__main__':
    try:
        # wait manager
        en_sim = rospy.get_param('en_sim', False)
        if not en_sim:
            rospy.sleep(3)
        else:
            rospy.sleep(5)

        pcmd = pose_cmd()
        pcmd.set_mode()
        rospy.sleep(.5)

        wait_moving()

    	while not rospy.is_shutdown():
            raw_input('[info] pess enter key ')
            run()
       	    rospy.Rate(10).sleep()
        sys.exit(0)

    except rospy.ROSInterruptException:
        rospy.loginfo('Pushiler is closed')
