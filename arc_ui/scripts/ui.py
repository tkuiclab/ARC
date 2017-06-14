#!/usr/bin/env python

"""
    ui.py - Version 0.1 2017-02-13

"""

import os

import rospy
import rospkg
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from arc_ui.srv import *


#const of config package name
Config_Pkg_Name = "config"
#const of teach file name
Teach_File_Name = "teach_mode.json"

def js_callback(msg):
    global now_joint
    now_joint = Twist()
    now_joint.linear.x = msg.position[0]
    now_joint.linear.y = msg.position[1]
    now_joint.linear.z = msg.position[2]
    now_joint.angular.x = msg.position[3]
    now_joint.angular.y = msg.position[4]
    now_joint.angular.z = msg.position[5]
    #print msg.position[0]

def eef_callback(msg):
    global  now_pose
    now_pose = Twist()
    now_pose.linear.x = msg.linear.x
    now_pose.linear.y = msg.linear.y
    now_pose.linear.z = msg.linear.z
    now_pose.angular.x = msg.angular.x
    now_pose.angular.y = msg.angular.y
    now_pose.angular.z = msg.angular.z
    #print now_pose

def handle_ui_server(req):
    cmd = req.cmd
    rospy.loginfo('Request cmd('+cmd+')')
    global last_pose
    last_pose = Twist()

    #now_pose = self.get_eef_pos()
    res = UI_ServerResponse()

    if  cmd=="Teach:EEF_Pose":
        res.pose = now_pose
        last_pose=now_pose
    elif cmd=="Teach:Shift_X":
        res.f = now_pose.linear.x-req.pose.linear.x
        a = now_pose.linear.x-last_pose.linear.x
        last_pose=now_pose
        print req.pose.linear.x
        #print res.f
        #print a
        #res.f = now_pose.linear.x-req.pose.linear.x
        #print req.float_ary
        #print req.pose
        #res.f = now_pose.linear.x - teach_get_pre_linear(req).x

    elif cmd=="Teach:Shift_Y":
        res.f = now_pose.linear.y-req.pose.linear.y
        a = now_pose.linear.y-last_pose.linear.y
        print req.pose.linear.y
        #print res.f
        #print a
        #print req.float_ary
        #print req.pose
        #res.f = now_pose.linear.y - teach_get_pre_linear(req).y

    elif cmd=="Teach:Shift_Z":
        res.f = now_pose.linear.z-req.pose.linear.z
        a = now_pose.linear.z-last_pose.linear.z
        print req.pose.linear.z
        #print req.float_ary
        #print req.pose
        #res.f = now_pose.linear.z - teach_get_pre_linear(req).z
    elif cmd=="Teach:SaveFile":
        rospy.loginfo('Save file to  ' + teach_file_path)
        f = open(teach_file_path, 'w')
        f.write(req.req_s)
        f.close()
    elif cmd=="Teach:ReadFile":
        rospy.loginfo('Read file From  ' + teach_file_path)
        f = open(teach_file_path, 'r')
        read_data = f.read()
        res.res_s = read_data
        f.close()


    res.result = "UI_Server Success (Process " + cmd + ")"
    return res


if __name__ == "__main__":
    rospy.init_node('ui_server')
    global eef_sub
    global teach_file_path

    rospack = rospkg.RosPack()
    teach_file_path = rospack.get_path(Config_Pkg_Name) + "/" +  Teach_File_Name

    eef_sub = rospy.Subscriber("eef_states",Twist,eef_callback)
    js_sub = rospy.Subscriber("/robotis/present_joint_states",JointState,js_callback)
    s = rospy.Service('ui_server', UI_Server, handle_ui_server)

    rospy.loginfo('ui.py READY!')

    rospy.spin()
