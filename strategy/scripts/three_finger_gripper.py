#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import serial
from strategy.srv import *

ser = None

def egripper_init():
    global ser
    if rospy.has_param(rospy.get_caller_id()+'/port'):
        port = rospy.get_param(rospy.get_caller_id()+'/port')
    else:
        port = '/dev/arc/three_gripper'
    if rospy.has_param(rospy.get_caller_id()+'/baud'):
        baud_rate = rospy.get_param(rospy.get_caller_id()+'/baud')
    else:
        baud_rate = 115200

    rospy.loginfo("Port is on %s", port)
    rospy.loginfo("Baud rate is %s", baud_rate)
    ser = serial.Serial(port, baud_rate, timeout=.1)

def handle_gripper_cmd(req):
    rospy.loginfo("Receive cmd = %s", req.gripper_cmd)
    ser.flushInput()
    ser.flushOutput()
    ser.write(req.gripper_cmd)
    rospy.sleep(.5)
    read_ser = ser.read()
    try:
        out_hex = "0x{:02x} ".format(ord(read_ser))
        rospy.loginfo("Is busy %s", out_hex)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    while out_hex != '0x00 ':
        read_ser = ser.read()
        try:
            out_hex = "0x{:02x} ".format(ord(read_ser))
            rospy.loginfo(" %s", out_hex)
            rospy.sleep(.5)
            ser.flushOutput()
            ser.flushInput()
        except rospy.ServiceException, e:
            print "Error: %s"%e
    res = GripperResponse(True)
    return res

def adaptive_gripper():
    rospy.init_node('adaptive_gripper', anonymous=False)
    egripper_init()
    rospy.Service('gripper_cmd', Gripper, handle_gripper_cmd)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    adaptive_gripper()