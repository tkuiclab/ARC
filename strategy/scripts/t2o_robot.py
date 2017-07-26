#! /usr/bin/env python
# pylint: disable = invalid-name
# pylint: disable = C0326, C0121, C0301
# pylint: disable = W0105, C0303, W0312
"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos, tan, pi
from numpy import multiply
import numpy

import rospy
import roslib; #roslib.load_manifest('obj_pose')
import tf


import actionlib
from std_msgs.msg import String, Float64

import obj_pose.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import json

import arm_task_rel
from gripper import *
from s import *

import stow_task
import LM_Control

obj_dis = 0.02 #0.015

class T2O:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        #self.__set_pubSub()
        self.Arm 			= arm_task_rel.ArmTask()
        rospy.on_shutdown(self.stop_task)
        self.__obj_pose_client = actionlib.SimpleActionClient("/obj_pose", obj_pose.msg.ObjectPoseAction)
        

    def stop_task(self):
        """Stop task running."""
        self.Arm.stop_task()
        
    #request object pose
    def obj_pose_request(self, obj):
        while self.Arm.busy:
            rospy.sleep(.1)
        rospy.loginfo('obj_pose_request() obj='+obj)
        
        #goal = obj_pose.msg.ObjectPoseGoal(obj)
        # goal = obj_pose.msg.ObjectPoseGoal(object_name = obj, limit_ary =[-0.14, 0.14, -0.1,  0.4, 0.35, 0.6])
        goal = obj_pose.msg.ObjectPoseGoal(object_name = '<Closest>', limit_ary =[-0.14, 0.14, -0.1,  0.4, 0.35, 0.6])
        
        self.__obj_pose_client.send_goal(goal,feedback_cb = self.obj_pose_feedback_cb, done_cb=self.obj_pose_done_cb )
        self.__obj_pose_client.wait_for_result()

    
    def safe_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.30, 0.00 , 0.3), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)

    def bin_photo_pose(self):
        # self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.3), (-110, 0, 0) )
        self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.2), (-100, 0, 0) )
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)

    def bin_place_pose(self):
        self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.1), (-90, 0, 0) )
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)


    def robot_photo_pose(self):
        
        self.Arm.pub_ikCmd('ptp', (0.45, 0.00 , 0.25), (-180, 0, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)
        gripper_suction_up()


    def robot_photo_pose_2(self):
        self.Arm.pub_ikCmd('ptp', (0.3, 0.00 , 0.25), (180, 180, 0))
        rospy.sleep(.5)
        while self.Arm.busy:
            rospy.sleep(.1)
        gripper_suction_up()


    def obj_pose_feedback_cb(self,fb):
        rospy.loginfo("In obj_pose_feedback_cb")
        rospy.loginfo("msg = " + fb.msg)
        rospy.loginfo("progress = " + str(fb.progress) + "% ")

    def obj_pose_done_cb(self, state, result):
        # self.arm_2_obj(result.object_pose)
        # self.tool_2_obj(result.object_pose)
        # self.tool_2_obj(result.object_pose, result.norm)
        # self.tool_2_obj(result.object_pose, result.norm, 180)
        # self.tool_2_obj2(result.object_pose, result.norm, 180, 20)
        # self.tool_2_obj_bin(result.object_pose, result.norm)
        self.tool_2_obj_bin(result.object_pose, result.norm, rel_pos = (0, 0, -0.2), rel_ang = (10, 0 , 0))
        # self.tool_2_obj_bin_straight(result.object_pose, result.norm)
        # self.tool_2_obj_bin2(result.object_pose, result.norm)

    def tool_2_obj2(self, obj_pose, norm, shot_deg = 0, robot_pitch = 0): #STOW
        relativeAng = robot_pitch

        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )
        rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        if (l.x * norm.x > 0) and (abs(norm.x) > abs(norm.y)) and (abs(norm.x) > abs(norm.z)) :
            print("#################\nInvert Norm.x\n#################")
            norm.x = norm.x*-1
            norm.y = norm.y*-1
            norm.z = norm.z*-1
            print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        if (abs(90 - r) <= 15):
            y = 0
            print("Forward Face y = " + str(new_y) + " -> " + str(0))

        print("(y, r)= (" + str(y) + ", " + str(r) + ")")
        
        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y_4_tote) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        rospy.loginfo("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]

        ###
        dis_real = math.sqrt(real_move_x*real_move_x + real_move_y*real_move_y + real_move_z*real_move_z)

        real_move_x_unit = real_move_x / dis_real
        real_move_y_unit = real_move_y / dis_real
        real_move_z_unit = real_move_z / dis_real

        dis = math.sqrt(real_move_z*real_move_z + real_move_y*real_move_y)*cos(radians(relativeAng))
        print("dis_real -> dis : "+str(dis_real)+' -> '+str(dis))
        detZ = abs(cam2tool_z - cam2tool_z*cos(radians(relativeAng)))
        detY = abs(cam2tool_z*sin(radians(relativeAng)))
        print("(detZ, detY) = ("+str(detZ)+", "+str(detY)+")")

        real_move_x_rot = real_move_x_unit
        real_move_y_rot = real_move_y_unit*cos(radians(relativeAng*cos(radians(shot_deg)))) - real_move_z_unit*sin(radians(relativeAng*cos(radians(shot_deg))))
        real_move_z_rot = real_move_z_unit*cos(radians(relativeAng*cos(radians(shot_deg)))) + real_move_y_unit*sin(radians(relativeAng*cos(radians(shot_deg))))

        real_move_x_rot = real_move_x_rot*dis
        real_move_y_rot = real_move_y_rot*(dis - detY)
        real_move_z_rot = real_move_z_rot*(dis - detZ)
        ###
        rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")
        rospy.loginfo("(real_move_x_rot, real_move_y_rot, real_move_z_rot)= (" + str(real_move_x_rot) + ", " + str(real_move_y_rot) + ", " + str(real_move_z_rot) + ")")

        #----------------Place---------------#
        self.Arm.relative_move_xyz_rot_pry(pitch = robot_pitch)
        #----------------Rotation---------------_#
        if y == 0 and shot_deg == 180 :
            self.Arm.move_2_Abs_Roll(y, blocking=True)
        else :
            self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r - relativeAng)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r-relativeAng)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_y_rot*-1)+', y = '+str(real_move_x_rot)+', z = '+str(real_move_z_rot*-1)+')')
        # return
        self.Arm.relative_xyz_base(x = real_move_y_rot*-1, y = real_move_x_rot, z = real_move_z_rot*-1)
        # self.Arm.relative_xyz_base(x = real_move_y_rot*-1, y = real_move_x_rot)

        rospy.loginfo('Move Angle Finish')

    def tool_2_obj(self, obj_pose, norm, shot_deg = 0): #STOW
        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )
        rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        if (l.x * norm.x > 0) and (abs(norm.x) > abs(norm.y)) and (abs(norm.x) > abs(norm.z)) :
            print("#################\nInvert Norm.x\n#################")
            norm.x = norm.x*-1
            norm.y = norm.y*-1
            norm.z = norm.z*-1
            print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        old_y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        old_r = 90 - (numpy.rad2deg(a.x) + 180)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        print("(old_y, old_r)= (" + str(old_y) + ", " + str(old_r) + ")")
        print("(y, r)= (" + str(y) + ", " + str(r) + ")")
        
        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y_4_tote) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        rospy.loginfo("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]

        rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")

        #----------------Rotation---------------_#
        self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_y*-1)+', y = '+str(real_move_x)+', z = '+str(real_move_z*-1)+')')

        self.Arm.relative_xyz_base(x = real_move_y*-1, y = real_move_x, z = real_move_z*-1)

        rospy.loginfo('Move Angle Finish')

    def tool_2_obj_bin(self, obj_pose, norm, rel_pos = (0, 0, -0.1), rel_ang = (10, 0 , 0)): # BIN
        relativeAng = rel_ang[0]

        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )
        rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        if (l.x * norm.x > 0) and (abs(norm.x) > abs(norm.y)) and (abs(norm.x) > abs(norm.z)) :
            print("#################\nInvert Norm.x\n#################")
            norm.x = norm.x*-1
            norm.y = norm.y*-1
            norm.z = norm.z*-1
            print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        if (abs(90 - r) <= 5) or abs(new_y) > 100:
            y = 0
            r = 90
            print("Forward Face y = "+str(new_y)+" -> "+str(0))
            print("Forward Face r = "+str(90 - (numpy.rad2deg(a.x) + 180))+" -> "+str(90))
            print("Forward Face Norm(x, y) = ("+str(norm.x)+", "+str(norm.y)+" -> (0, 0)")
            norm.x = 0
            norm.y = 0

        print("(y, r)= (" + str(y) + ", " + str(r) + ")")

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))
        move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        print("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*(obj_dis), norm.y*(obj_dis), norm.z*(obj_dis)]

        real_move_x = move_cam_x + obj_distance[0]
        real_move_y = move_cam_y + obj_distance[1]
        real_move_z = move_cam_z + obj_distance[2]
        ###
        dis_real = math.sqrt(real_move_x*real_move_x + real_move_y*real_move_y + real_move_z*real_move_z)

        real_move_x_unit = real_move_x / dis_real
        real_move_y_unit = real_move_y / dis_real
        real_move_z_unit = real_move_z / dis_real

        # dis = abs(real_move_z)*cos(radians(relativeAng))
        dis = math.sqrt(real_move_z*real_move_z + real_move_y*real_move_y)*cos(radians(relativeAng))
        print("dis_real -> dis : "+str(dis_real)+' -> '+str(dis))
        detZ = abs(cam2tool_z - cam2tool_z*cos(radians(relativeAng)))
        detY = abs(cam2tool_z*sin(radians(relativeAng)))
        print("(detZ, detY) = ("+str(detZ)+", "+str(detY)+")")

        real_move_x_rot = real_move_x_unit
        real_move_y_rot = real_move_y_unit*cos(radians(relativeAng)) - real_move_z_unit*sin(radians(relativeAng))
        real_move_z_rot = real_move_z_unit*cos(radians(relativeAng)) + real_move_y_unit*sin(radians(relativeAng))

        real_move_x_rot = real_move_x_rot*dis
        real_move_y_rot = real_move_y_rot*(dis - detY)
        real_move_z_rot = real_move_z_rot*(dis - detZ)
        ###
        rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")
        rospy.loginfo("(real_move_x_rot, real_move_y_rot, real_move_z_rot)= (" + str(real_move_x_rot) + ", " + str(real_move_y_rot) + ", " + str(real_move_z_rot) + ")")

        #----------------Place---------------#
        # self.bin_place_pose()
        # self.Arm.relative_xyz_base(x = rel_pos[0], y = rel_pos[1], z = rel_pos[2])
        task.Arm.relative_move_xyz_rot_pry(x = rel_pos[0], y = rel_pos[1], z = rel_pos[2], pitch = rel_ang[0], roll = rel_ang[1], yaw = rel_ang[2])

        #----------------Rotation---------------_#
        self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r - relativeAng)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r-relativeAng)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z_rot)+', y = '+str(real_move_x_rot)+', z = '+str(real_move_y_rot*-1)+')')
        # return
        self.Arm.relative_xyz_base(x = real_move_z_rot, y = real_move_x_rot, z = real_move_y_rot*-1)
        # self.Arm.relative_xyz_base(x = 0.15)
        # self.Arm.relative_xyz_base(y = real_move_x_rot, z = real_move_y_rot*-1)

        rospy.loginfo('Move Angle Finish')

    def tool_2_obj_bin_straight(self, obj_pose, norm, shot_deg = 0): # BIN
        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )
        rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        if (l.x * norm.x > 0) and (abs(norm.x) > abs(norm.y)) and (abs(norm.x) > abs(norm.z)) :
            print("#################\nInvert Norm.x\n#################")
            norm.x = norm.x*-1
            norm.y = norm.y*-1
            norm.z = norm.z*-1
            print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        old_y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        old_r = 90 - (numpy.rad2deg(a.x) + 180)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        print("(old_y, old_r)= (" + str(old_y) + ", " + str(old_r) + ")")
        print("(y, r)= (" + str(y) + ", " + str(r) + ")")

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        rospy.loginfo("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]

        rospy.loginfo("(y, r) = (" + str(y) + ", " + str(r) + ")")
        rospy.loginfo("(ori_move_cam_x, ori_move_cam_y, ori_move_cam_z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")
        
        #----------------Rotation---------------_#
        self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r)+')')
        # print('self.Arm.relative_xyz_base(x = '+str(real_move_y*-1)+', y = '+str(real_move_x)+', z = '+str(real_move_z*-1)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z)+', y = '+str(real_move_x)+', z = '+str(real_move_y*-1)+')')

        # return

        # if real_move_y > 0.026 :
        #     print('\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/')
        #     real_move_y = 0.026

        self.Arm.relative_xyz_base(x = real_move_z, y = real_move_x, z = real_move_y*-1)

        rospy.loginfo('tool_2_obj_bin_strasight Finish')

    def tool_2_obj_bin2(self, obj_pose, norm, shot_deg = 0): # BIN Straight Yaw Move
        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")")
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )
        rospy.loginfo("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        # if l.x * norm.x > 0 :
        #     print("#################\nInvert Norm.x\n#################")
        #     norm.x = norm.x*-1
        #     norm.y = norm.y*-1
        #     norm.z = norm.z*-1
        #     print("(norm.x, norm.y, norm.z)= (" + str(norm.x) + ", " + str(norm.y)+ ", " + str(norm.z) + ")")

        new_y = numpy.angle(complex(norm.y*-1, norm.x), deg = True)
        old_y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        old_r = 90 - (numpy.rad2deg(a.x) + 180)
        y = new_y
        r = 90 - (numpy.rad2deg(a.x) + 180)

        print("(old_y, old_r)= (" + str(old_y) + ", " + str(old_r) + ")")
        print("(y, r)= (" + str(y) + ", " + str(r) + ")")

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        rospy.loginfo("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]
        ###
        dis_real = math.sqrt(real_move_x*real_move_x + real_move_y*real_move_y + real_move_z*real_move_z)

        real_move_x_unit = real_move_x / dis_real
        real_move_y_unit = real_move_y / dis_real
        real_move_z_unit = real_move_z / dis_real

        # dis = abs(real_move_z)*cos(radians(20))
        dis = math.sqrt(real_move_x*real_move_x + real_move_y*real_move_y)*cos(radians(relativeAng))
        print("dis = "+str(dis))
        detZ = abs(cam2tool_z - cam2tool_z*cos(radians(20)))
        detX = abs(cam2tool_z*sin(radians(20)))
        print("(detZ, detX) = ("+str(detZ)+", "+str(detX)+")")

        real_move_x_rot = real_move_x_unit*cos(radians(20)) + real_move_z_unit*sin(radians(20))
        real_move_y_rot = real_move_y_unit
        real_move_z_rot = real_move_x_unit*sin(radians(20)) + real_move_z_unit*sin(radians(20))

        real_move_x_rot = real_move_x_rot*(dis - detX)
        real_move_y_rot = real_move_y_rot*dis
        real_move_z_rot = real_move_z_rot*(dis - detZ)
        ###
        rospy.loginfo("(y, r) = (" + str(y) + ", " + str(r) + ")")
        rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")

        #----------------Place---------------#
        self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.25), (-90, 0, 0) )

        #----------------Rotation---------------_#
        self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z)+', y = '+str(real_move_x)+', z = '+str(real_move_y*-1)+')')

        # return

        if real_move_y > 0.026 :
            print('\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/')
            real_move_y = 0.026

        self.Arm.relative_xyz_base(x = real_move_z, y = real_move_x, z = real_move_y*-1)

        rospy.loginfo('tool_2_obj_bin_straight Finish')



if __name__ == '__main__':

    rospy.init_node('t2o_robot', anonymous=True)

    task = T2O()
    rospy.sleep(0.5)
    rospy.loginfo('T2O Ready')

    # task.Arm.relative_xyz_base(x = -0.2)
    # task.safe_pose()
    # -------Back 2 home------#.
    # task.safe_pose()
    # task.Arm.home()
    # exit()

    #----------- Go Photo Pose--------#
    # task.robot_photo_pose()
    
    # while task.Arm.busy:
    #     rospy.sleep(.1)
    
    # s = Strategy()
    # s.stow.LM_2_tote()

    # STOW TEST
    # arm = arm_task_rel.ArmTask()
    # lm = LM_Control.CLM_Control()
    # stow = StowTask(arm, lm)
    # stow.LM_2_tote()
    # stow.arm_photo_pose_2()
    # stow.arm_photo_pose()
    # gripper_suction_up()
    # task.obj_pose_request('robots_dvd')

    # PICK TEST
    arm = arm_task_rel.ArmTask()
    lm = LM_Control.CLM_Control()
    s = PickTask(arm, lm)
    s.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', 'a') + 18000)
    rospy.sleep(0.3)
    s.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', 'a'))
    task.Arm.pub_ikCmd('ptp', (0.2, 0.0 , 0.4), (-100, 0, 0))
    task.obj_pose_request('colgate_toothbrush_4pk')

    ### Bin Place ###
    # s = Strategy()
    # s.test_go_bin_LM('h')
    # s.stow.LM_2_tote()
    # task.bin_photo_pose()
    # task.Arm.relative_xyz_base(x = -0.2)
    # task.bin_place_pose()
    ### New
    # task.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.3), (-100, 0, 0) )
    # gripper_suction_up()
    # task.obj_pose_request('hanes_socks')
    # task.obj_pose_request('poland_spring_water')
    # task.Arm.relative_move_xyz_rot_pry(pitch = 10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print 'ok...'
        r.sleep()
