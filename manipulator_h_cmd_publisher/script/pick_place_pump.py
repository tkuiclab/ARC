#!/usr/bin/env python

''' This is pick and placerun demo script '''

import sys
import copy
import rospy

from kinematics_cmd import pose_cmd, pose_type
from arduino_pub_pump import com_arduino

def wait_moving():
    ''' waiting arm moving '''
    try:
        while pcmd.arm_is_moving():
            rospy.Rate(10).sleep()
    except Exception:
        pass

def send_cmd(cmd):
    ''' publishing cmd message '''
    wait_moving()
    pcmd.publish_cmd(pcmd.cmd_to_kinematics_msg(cmd))

def ctrl_pump(switch):
    ''' control pump '''
    wait_moving()
    arduino.pump_ctrl(switch)
    rospy.sleep(.2)

def pick_place(pick, place, rot=0.0):
    ''' going to four positions and take object using pump '''
    pick_up = copy.deepcopy(pick)
    pick_up.position[2] += 0.06      # z
    send_cmd(pick_up)
    send_cmd(pick)
    ctrl_pump(True)
    send_cmd(pick_up)

    place_up = copy.deepcopy(place)
    place_up.position[2] += 0.06     # z
    send_cmd(place_up)
    send_cmd(place)
    ctrl_pump(False)
    send_cmd(place_up)

def run():
    ''' run demo cmd '''
    pick = pose_type([0.2, -0.3, 0.06], [0.0, 90.0, 0.0])
    place = pose_type([0.3, 0.0, 0.06], [0.0, 90.0, 0.0])
    place2 = pose_type([0.3, 0.3, 0.06], [0.0, 90.0, 0.0])
    place3 = pose_type([0.0, -0.3, 0.06], [0.0, 90.0, 0.0])

    #pick_place(pick, place)
    #pick_place(pick, place2)
    #pick_place(pick, place3)
    pick_place(place3, pick)

if __name__ == '__main__':
    try:
        arduino = com_arduino()

        en_sim = rospy.get_param('en_sim', False)
        if not en_sim:
            rospy.sleep(3)
        else:
            rospy.sleep(5)

        pcmd = pose_cmd()
        pcmd.set_mode()
        rospy.sleep(.5)
        pcmd.initial_pose()
        wait_moving()

        # Ready pose
        send_cmd( pose_type([0.3, 0.0, 0.15], [0.0, 90.0, 0.0]) )
        
        while not rospy.is_shutdown():
            run()
            rospy.Rate(10).sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo('Pushiler is closed')
