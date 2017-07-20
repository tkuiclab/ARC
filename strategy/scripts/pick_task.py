#!/usr/bin/env python

"""Strategy for picking task."""

from __future__ import print_function
import math
from math import radians, degrees, sin, cos, tan, pi
import numpy
import rospy
import actionlib

import obj_pose.msg
from darkflow_detect.srv import Detect
from strategy.srv import *
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String, ByteMultiArray
from actionlib_msgs.msg import GoalID, GoalStatusArray

import LM_Control
import arm_task_rel
from config import *
from gripper import *
from task_parser import *
from get_obj_info import order_boxes
from object_distribution import bin_dict

# Define State jmp
WaitTask = 1        # Wait Task
ParseJSON = 2       # Parse Json
Down2Pick = 3       # Move down to pick object in bin.
Init_Pos = 4        # Make robot arm go to the initial pos()
Move2BinFront = 5   # Make robot arm go to the specify bin
WaitRobot = 6       # wait for robot complete task
Up2LeaveBin = 7     # Move up to leave bin (robot arm still in bin)
LeaveBin = 8        # Make robot arm leave bin
FinishTask = 9
RobotMove2Bin = 10
PickObj = 11
PlaceObj = 12
RobotMove2Box = 13
Move2PlaceObj2 = 14
Prepare2Place = 15
Down2Place = 16
Recover2InitPos = 17
NeetStep = 18
LeavedBin = 19
LeaveBox = 20

PhotoPose = 26
VisionProcess = 27
Rotate2PickPose = 28
WaitVision = 29
CheckIsHold = 30

_REL_GO_BIN_FRONT = .1
_BIN_MAX_DOWN = .135

obj_dis = 0.02

# For checking vacuum function
check_next_states = [
    RobotMove2Box,
    Prepare2Place,
    Down2Place
]

_LEFT_SHIFT = LM_Right_Arm_Shift


class PickTask:
    """Picking task object."""

    def __init__(self, arm, lm):
        """Initial for object."""
        rospy.loginfo('PickTask::init()')
        self.__obj_pose_client = actionlib.SimpleActionClient(
            "/obj_pose", obj_pose.msg.ObjectPoseAction)
        self.__if_suck_sub = rospy.Subscriber(
            '/IfSuck', ByteMultiArray, self.ifsuck_cb, queue_size=1)
        self.Arm = arm
        self.LM = lm
        self.var_init()

    def obj_pose_feedback_cb(self, fb):
        """Feedback callback for obj_pose."""
        rospy.loginfo('In obj_pose_feedback_cb')
        rospy.loginfo('msg = {}'.format(fb.msg))
        rospy.loginfo('progress = {} %'.format(fb.progress))

    def obj_pose_done_cb(self, state, result):
        """Response callback for obj_pose."""
        if not result.success:
            self.obj_pose_unsuccess()
        else:
            print('CurrentTask:', self.now_pick.item, 'Closest:', result.object_name)
            if result.object_name == self.now_pick.item:
                self.obj_pose_success(result)
            else:
                print('Search other item in same bin')
                # Search other item in same bin
                for i, task in enumerate(self.pick_list):
                    print('Searching...')
                    if (task.from_bin.lower() == self.bin and
                            task.item == result.object_name):
                        # self.state = VisionProcess
                        self.pick_list.insert(i+1, self.now_pick)
                        self.pick_get_one(self.pick_list, i)
                        print('Current item:', self.now_pick.item)
                        self.obj_pose_success(result)
                        return
                self.cover_list.append(self.now_pick)
                self.state = FinishTask

    def obj_pose_success(self, result):
        self.state = self.next_state
        self.obj_pose_ret = result
        print('obj_pose {}'.format(self.obj_pose_ret.object_pose))

    def obj_pose_unsuccess(self):
        rospy.logwarn('ROI Fail!! obj -> {}'.format(self.now_pick.item))
        self.state = FinishTask
        self.obj_pose_ret = None
        # Add the item to fail list
        self.fail_list.append(self.now_pick)

    def get_bin_dims(self, bin):
        bin_id = ord(bin) - ord('a')
        # dims of the box
        return bin_dict[bin_id].L, bin_dict[bin_id].W, bin_dict[bin_id].H

    def request_highest_item(self):
        if len(self.detect_all_in_bin):
            W, _, H = self.get_bin_dims(self.bin)
            goal = obj_pose.msg.ObjectPoseGoal(
                object_name="<Closest>",
                object_list=self.detect_all_in_bin,
                # [xmin, xmax, ymin, ymax, zmin, zmax]
                limit_ary=[-W/2.0, W/2.0, -H/2.0, H/2.0, 0.3, 0.6]
            )
            self.__obj_pose_client.send_goal(
                    goal,
                    feedback_cb=self.obj_pose_feedback_cb, 
                    done_cb=self.obj_pose_done_cb
            )
            return True
        return False

    def get_detect_all_list(self):
        detect_client = rospy.ServiceProxy('/detect', Detect)
        res = detect_client("all")
        return res.detected

    def gen_detect_all_in_bin(self):
        # Detection all of object
        detect_all_list = self.get_detect_all_list()

        curbin = None
        for bin in self.item_loc['bins']:
            if bin['bin_id'] == self.now_pick.from_bin:
                curbin = bin
                break

        # Check object in bin
        self.detect_all_in_bin = []
        for d_item in detect_all_list:
            for bin_item in curbin['contents']:
                if d_item.object_name == bin_item:
                    self.detect_all_in_bin.append(d_item.object_name)

    def print_task_list(self):
        print('---------------------------------------------')
        print('pick_list')
        for i, task in enumerate(self.pick_list):
            print(i, task.item, task.from_bin, task.to_box)
        print('cover_list')
        for i, task in enumerate(self.cover_list):
            print(i, task.item, task.from_bin, task.to_box)
        print('fail_list')
        for i, task in enumerate(self.fail_list):
            print(i, task.item, task.from_bin, task.to_box)
        print('success_list')
        for i, item in enumerate(self.success_list):
            print(i, item)
        print('---------------------------------------------')

    def ifsuck_cb(self, res):
        """suction status callback."""
        self.suck_ary = res.data

    @property
    def suck_num(self):
        return sum(self.suck_ary)

    @property
    def finish(self):
        return self.state == WaitTask

    def is_ready(self):
        return self.pick_list is not None

    def convert_box_to_char(self, box_name):
        try:
            num = order_boxes.index(box_name)
            for i, id in enumerate(['a', 'b', 'c']):
                if num == i:
                    return id
        except ValueError as e:
            print(e)
        return 'a'

    def pick_get_one(self, which_list, index=0):
        """Get one of pick task and set bin, box."""
        if not len(which_list):
            return False

        self.now_pick = which_list.pop(index)
        self.bin = self.now_pick.from_bin.lower()
        self.item = self.now_pick.item
        self.box = self.convert_box_to_char(self.now_pick.to_box)
        return True

    def run(self):
        self.pick_list = list(self.pick_source)
        self.pick_get_one(self.pick_list)
        self.state = RobotMove2Bin
        self.print_task_list()

    def pick_core(self):
        """Main procedure of finite state machine."""
        self.update_status()

        if self.state == WaitTask:
            return

        elif self.state == WaitVision:
            return

        # Move to bin and initial pose
        elif self.state == RobotMove2Bin:
            self.info = "(GoBin) Robot Move to Bin {}".format(self.bin)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = PhotoPose

            # Move to bin using linear mobile
            self.Is_BaseShiftOK = False
            self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', self.bin) + _LEFT_SHIFT)
            rospy.sleep(0.3)
            self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', self.bin))
            # Arm move to initial pose
            gripper_suction_up()
            self.Arm.pub_ikCmd('ptp', (0.3, 0.0, 0.3), (-90, 0, 0))
            # self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.3), (-90, 0, 0))

        elif self.state == Init_Pos:
            self.info = "(GoBin) Arm To Init_Pos"
            print(self.info)

            # Change state
            self.state = WaitRobot
            # self.next_state = PhotoPose
            gripper_suction_up()
            self.next_state = VisionProcess

            self.Arm.pub_ikCmd('ptp', (0.3, 0.0, 0.3), (-90, 0, 0))

        elif self.state == PhotoPose:
            self.info = "(GoBin) Arm to PhotoPose"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = VisionProcess

            gripper_suction_up()
            # self.Arm.pub_ikCmd('ptp', (0.35, 0.0 , 0.35), (-100, 0, 0))
            self.Arm.pub_ikCmd('ptp', (0.2, 0.0 , 0.4), (-100, 0, 0))

        elif self.state == VisionProcess:
            self.info = "(Vision) Request Highest"
            print(self.info)

            self.gen_detect_all_in_bin()
            print("detect_all_in_bin_list[] -> " + str(self.detect_all_in_bin))

            # ActionLib request
            if self.request_highest_item():
                # Change state
                self.state = WaitVision
                self.next_state = NeetStep
            else:
                self.obj_pose_unsuccess()

        elif self.state == NeetStep:
            self.info = "(Catch) Arm Move to Bin Front {}".format(self.bin)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = PickObj

            if self.obj_pose_ret is not None:
                # self.tool_2_obj_bin_straight(self.obj_pose_ret.object_pose, self.obj_pose_ret.norm)
                self.suction_angle = self.tool_2_obj_bin(
                    self.obj_pose_ret.object_pose, self.obj_pose_ret.norm)

        # Arm move to bin front
        elif self.state == Move2BinFront:
            self.info = "(Catch) Arm Move to Bin Front {}".format(self.bin)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Rotate2PickPose

            self.Arm.relative_move_nsa(a=_REL_GO_BIN_FRONT)

        # Tune pose to fit object
        elif self.state == Rotate2PickPose:
            self.info = "(Catch) Arm Rotate 2 Pick Pose"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = PickObj

            tool_x = self.obj_pose_ret.object_pose.linear.x
            tool_y = self.obj_pose_ret.object_pose.linear.y
            # Fixed height to avoid collision
            if tool_y > _BIN_MAX_DOWN:
                tool_y = _BIN_MAX_DOWN
            print("tool_y: {}".format(tool_y))
            self.Arm.relative_move_nsa(s=tool_x, n=tool_y)

        # Enable the vacuum
        elif self.state == PickObj:
            self.info = "(Catch) Vacuum Enable"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Down2Pick

            gripper_vaccum_on()
            rospy.sleep(.1)

        # Pick the object
        elif self.state == Down2Pick:
            self.info = "(Catch) Arm Down to Catch"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Up2LeaveBin

            # self.Arm.relative_move_suction('ptp', self.suction_angle, (obj_dis + 0.01))
            if not self.suck_num:
                self.Arm.relative_move_suction('ptp', self.suction_angle, (obj_dis))

        # Move upper
        elif self.state == Up2LeaveBin:
            self.info = "(Catch) Arm Up"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = LeaveBin

            self.Arm.relative_xyz_base(z=0.03)

        # Move out of bin
        elif self.state == LeaveBin:
            self.info = "(Catch) Arm Leave Bin"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = RobotMove2Box

            # self.Arm.relative_move_nsa(a=-0.05)
            self.Arm.relative_xyz_base(x=-.18)

        # Move out of bin
        elif self.state == LeavedBin:
            self.info = "(Catch) Arm Leaved Bin"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = RobotMove2Box

            self.Arm.pub_ikCmd('ptp', (0.3, 0, 0.3), (-90, 0, 0))
            # self.Arm.relative_xyz_base(x=-.1)

        # Move to target box
        elif self.state == RobotMove2Box:
            self.info = "(GoBox) Robot Move to Box {}".format(self.box)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Prepare2Place

            self.Is_BaseShiftOK = False
            self.LM.pub_LM_Cmd(2, GetShift('Box', 'x', self.box))
            rospy.sleep(0.3)
            self.LM.pub_LM_Cmd(1, GetShift('Box', 'z', self.box))
            self.Arm.pub_ikCmd('ptp', (0.25, 0, 0.3), (-90, 0, 0))
            # self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.2), (-180, 0, 0))

        # Prepare to place
        elif self.state == Prepare2Place:
            self.info = "(GoBox) Arm Prepare to Place {}".format(self.box)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Down2Place

            self.Arm.pub_ikCmd('ptp', (0.35, -.05, 0.2), (-180, -90, 0))

        # Move lower
        elif self.state == Down2Place:
            self.info = "(GoBox) Arm Down to Place {}".format(self.box)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = PlaceObj

            self.Arm.relative_move_nsa(a=.15)

        # Disable the vacuum
        elif self.state == PlaceObj:
            self.info = "(GoBox) Vaccum Disable - [Success]"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = LeaveBox

            gripper_vaccum_off()
            self.update_location_file()

        # Leave box
        elif self.state == LeaveBox:
            self.info = "(GoBox) Leave Box"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = FinishTask

            self.Arm.relative_move_nsa(a=-.15)

        elif self.state == Recover2InitPos:
            self.info = "Arm Recover2InitPos"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = FinishTask

            self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.22), (-90, 0, 0))

        # ===============================================================

        # Wating robot moving
        elif self.state == WaitRobot:
            change_next_state = False
            if not self.Is_BaseShiftOK and self.Is_LMArrive and not self.Is_LMBusy:
                self.Is_BaseShiftOK = True
                # self.state = self.next_state
                print('LM Positive trigger')
            elif self.Is_BaseShiftOK and not self.Is_ArmBusy:
                # self.state = self.next_state
                change_next_state = True
                print('Robot Move Done')

            if change_next_state:
                if self.check_vaccum_by_next_state(self.next_state):
                    self.state = self.next_state
                # Grasping nothing
                else:
                    gripper_vaccum_off()
                    self.state = FinishTask

        # One of task is finished
        elif self.state == FinishTask:
            self.Is_BaseShiftOK = False
            self.state = RobotMove2Bin

            # Can get next task from pick list
            if self.pick_get_one(self.pick_list):
                self.info = "Finish Pick One Object"
            # elif self.pick_get_one(self.fail_list):
            #     self.info = "Finish Pick One Object and Execute Fail list"
            else:
                self.task_finish()
                self.info = "Finish Pick All of Task"
            print(self.info)
            self.print_task_list()

    def check_vaccum_by_next_state(self, n_s):
        """Checking status of vacuum is hold."""
        # These state need to check vacuum is hold
        if n_s in check_next_states:
            self.info = "(Check) Status of Suction: {}".format(self.suck_num)
            print(self.info)
            if not self.suck_num:
                # Add the item to fail list
                self.fail_list.append(self.now_pick)
            return self.suck_num > 0
        # Other state do not
        return True

    def update_location_file(self):
        """Update item location file and save."""
        rospy.loginfo('Update item location file')
        # Remove item in bin
        for bin in self.item_loc['bins']:
            if bin['bin_id'] == self.now_pick.from_bin:
                bin['contents'].remove(self.now_pick.item)
                break
        # Append item to box
        for box in self.item_loc['boxes']:
            if box['size_id'] == self.now_pick.to_box:
                box['contents'].append(self.now_pick.item)
                break
        # Savie item location file
        write_item_location(self.item_loc, filetype='Pick')
        self.success_list.append(self.now_pick.item)

    def get_info(self):
        """Return json information."""
        return {
            'info': self.info,
            'item': self.item,
            'bin': self.bin,
            'box': self.now_pick.to_box
        }

    def var_init(self):
        """Initial all of variables."""
        self.pick_source = None
        self.pick_list = None
        self.success_list = list()
        self.cover_list = list()
        self.fail_list = list()

        self.item_location = None
        self.order = None
        self.item_loc = None
        # === Initialize State ===
        self.state = WaitTask
        self.past_state = WaitTask
        self.next_state = WaitTask
        self.Is_ArmBusy = False
        self.Is_LMBusy = False
        self.Last_LM_Busy = False
        self.Is_LMArrive = True
        self.Last_LMArrive = True
        self.Is_BaseShiftOK = False
        self.Is_ArmMoveOK = False

        self.bin = 'e'
        self.box = 'a'

        self.suction_angle = None

    def update_status(self):
        """Update status of arm and linear mobile."""
        self.Last_LM_Busy = self.Is_LMBusy
        self.Last_LMArrive = self.Is_LMArrive
        self.Is_ArmBusy = self.Arm.busy
        self.Is_LMBusy = self.LM.IsBusy
        self.Is_LMArrive = self.LM.IsArrive

    def task_finish(self):
        """Setting variables for finish."""
        rospy.loginfo('Finish Task')
        self.fail_list = list()

        self.state = WaitTask
        self.next_state = WaitTask
        self.Is_ArmBusy = False
        self.Is_LMBusy = False
        self.Last_LM_Busy = False
        self.Is_LMArrive = True
        self.Last_LMArrive = True
        self.Is_BaseShiftOK = False

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
        
        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        r = 90 - (numpy.rad2deg(a.x) + 180)

        rospy.loginfo("(real_yaw, real_roll)= (" + str(y) + ", " + str(r) + ")")

        # move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        # move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        # move_cam_z = l.z - (gripper_length*cos(radians(r))) - 0.1#cam2tool_z

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + 0.04) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
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

        self.Arm.relative_xyz_base(x = 0.06)
        self.Arm.relative_xyz_base(x = real_move_z - 0.06, y = real_move_x, z = real_move_y*-1)
        # self.Arm.relative_xyz_base(y = real_move_x, z = real_move_y*-1)
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z)+', y = '+str(real_move_x)+', z = '+str(real_move_y*-1)+')')

        gripper_vaccum_on()

        # suction move
        self.Arm.relative_move_suction('ptp', r, obj_dis + 0.02)
        print("self.Arm.relative_move_suction('ptp', "+str(r)+", obj_dis + 0.02)")
        print("=====")

        # rospy.sleep(3)
        # gripper_vaccum_off()
        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        self.Arm.relative_move_suction('ptp', r, (obj_dis + 0.02)*-1)
        rospy.loginfo('tool_2_obj_bin_straight Finish')
    '''
    def tool_2_obj_bin(self, obj_pose, norm, shot_deg = 0): # BIN
        p = obj_pose
        a = p.angular
        l = p.linear

        rospy.loginfo("object_pose")
        rospy.loginfo("(x,y,z)= (" + str(l.x) + ", " + str(l.y)+ ", " + str(l.z) + ")") 
        rospy.loginfo("(roll,pitch,yaw)= (" 
                        + str(numpy.rad2deg(a.x)) + ", " 
                        + str(numpy.rad2deg(a.y)) + ", " 
                        + str(numpy.rad2deg(a.z)) + ")" )

        if (l.x ==0 and l.y==0 and l.z==0) or l.z < 0:
            return

        y = (numpy.rad2deg(a.z) - 180) if numpy.rad2deg(a.z) > 0  else (numpy.rad2deg(a.z) + 180)
        r = 90 - (numpy.rad2deg(a.x) + 180)

        print("(y, r)= (" + str(y) + ", " + str(r) + ")")

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))*cos(radians(shot_deg))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        print("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

        real_move_x = move_cam_x + obj_distance[0]*cos(radians(shot_deg))
        real_move_y = move_cam_y + obj_distance[1]*cos(radians(shot_deg))
        real_move_z = move_cam_z + obj_distance[2]
        ###
        dis_real = math.sqrt(real_move_x*real_move_x + real_move_y*real_move_y + real_move_z*real_move_z)

        real_move_x_unit = real_move_x / dis_real
        real_move_y_unit = real_move_y / dis_real
        real_move_z_unit = real_move_z / dis_real

        dis = abs(real_move_z)*cos(radians(20))
        # dis = 0.2 / tan(radians(20))
        print("dis = "+str(dis))
        detZ = abs(cam2tool_z - cam2tool_z*cos(radians(20)))
        detY = abs(cam2tool_y*sin(radians(20)))
        print("(detZ, detY) = ("+str(detZ)+", "+str(detY)+")")

        real_move_x_rot = real_move_x_unit
        real_move_y_rot = real_move_y_unit*cos(radians(20)) - real_move_z_unit*sin(radians(20))
        real_move_z_rot = real_move_z_unit*cos(radians(20)) + real_move_y_unit*sin(radians(20))

        real_move_x_rot = real_move_x_rot*dis
        real_move_y_rot = real_move_y_rot*(dis - detY)
        real_move_z_rot = real_move_z_rot*(dis - detZ)
        ###
        rospy.loginfo("(l.x, l.y, l.z)= (" + str(l.x) + ", " + str(l.y) + ", " + str(l.z) + ")")
        rospy.loginfo("(move_cam_x, move_cam_y, move_cam_z)= (" + str(move_cam_x) + ", " + str(move_cam_y) + ", " + str(move_cam_z) + ")")
        rospy.loginfo("(real_move_x, real_move_y, real_move_z)= (" + str(real_move_x) + ", " + str(real_move_y) + ", " + str(real_move_z) + ")")
        rospy.loginfo("(real_move_x_rot, real_move_y_rot, real_move_z_rot)= (" + str(real_move_x_rot) + ", " + str(real_move_y_rot) + ", " + str(real_move_z_rot) + ")")

        #----------------Place---------------#
        self.Arm.pub_ikCmd('ptp', (0.4, 0.0 , 0.2), (-90, 0, 0) )
        
        #----------------Rotation---------------#
        self.Arm.relative_rot_nsa(roll = y)
        gripper_suction_deg(r-20)

        # if real_move_y_rot > 0.026:
        #     print('\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/\FOR SAFE/')
        #     real_move_y_rot = 0.026

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r-20)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z_rot)+', y = '+str(real_move_x_rot)+', z = '+str(real_move_y_rot*-1)+')')

        self.Arm.relative_xyz_base(x = real_move_z_rot, y = real_move_x_rot, z = real_move_y_rot*-1)
        # self.Arm.relative_xyz_base(y = real_move_x_rot, z = real_move_y_rot*-1)

        gripper_vaccum_on()

        self.Arm.relative_move_suction('ptp', r, obj_dis + 0.02)
        print("self.Arm.relative_move_suction('ptp', "+str(r)+", obj_dis + 0.018)")
        print("=====\n")

        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        if self.suck_num == 0:
            self.Arm.relative_move_suction('ptp', r, 0.01)

        #----------------Return---------------_#
        self.Arm.relative_move_suction('ptp', r, (obj_dis + 0.02)*-1)
        rospy.loginfo('tool_2_obj_bin Finish')
    '''

    def tool_2_obj_bin(self, obj_pose, norm, rel_pos = (0, 0, -0.2), rel_ang = (10, 0 , 0)): # BIN
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

        if (abs(90 - r) <= 15):
            y = 0
            print("Forward Face y = " + str(new_y) + " -> " + str(0))

        print("(y, r)= (" + str(y) + ", " + str(r) + ")")

        move_cam_x = (l.x - (gripper_length*sin(radians(y)))*sin(radians(r)))
        move_cam_y = ((l.y + cam2center_y) + (gripper_length*cos(radians(y)))*sin(radians(r)))
        move_cam_z = l.z - (gripper_length*cos(radians(r))) - cam2tool_z

        print("NORMAL(x, y, z) = (" + str(norm.x) + ", " + str(norm.y) + ", " + str(norm.z) +")")
        obj_distance = [norm.x*obj_dis, norm.y*obj_dis, norm.z*obj_dis]

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
        self.Arm.relative_move_xyz_rot_pry(x = rel_pos[0], y = rel_pos[1], z = rel_pos[2], pitch = rel_ang[0], roll = rel_ang[1], yaw = rel_ang[2], blocking=True)

        #----------------Rotation---------------_#
        self.Arm.relative_rot_nsa(roll = y, blocking=True)
        gripper_suction_deg(r - relativeAng)

        print('=====')
        print('self.Arm.relative_rot_nsa(roll = '+str(y)+')')
        print('self.Arm.gripper_suction_deg('+str(r-relativeAng)+')')
        print('self.Arm.relative_xyz_base(x = '+str(real_move_z_rot)+', y = '+str(real_move_x_rot)+', z = '+str(real_move_y_rot*-1)+')')
        # return
        self.Arm.relative_xyz_base(x = real_move_z_rot, y = real_move_x_rot, z = real_move_y_rot*-1, blocking=True)

        rospy.loginfo('Move Angle Finish')
        return r


def _test():
    """Testing this module."""
    try:
        rospy.init_node('picking_strategy', anonymous=True, disable_signals=True)
        arm = arm_task_rel.ArmTask()
        lm = LM_Control.CLM_Control()
        s = PickTask(arm, lm)

        # s.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', 'a') + _LEFT_SHIFT)
        # rospy.sleep(0.3)
        # s.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', 'a'))

        # s.Arm.pub_ikCmd('ptp', (0.2, 0.0 , 0.4), (-100, 0, 0))
        # return

        # Setting picking list
        s.pick_source, s.item_loc = read_pick_task_and_location()

        rospy.sleep(0.3)
        s.run()

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown() and not s.finish:
            s.pick_core()
            rate.sleep()
    except KeyboardInterrupt as e:
        print('\nKeyboardInterrupt exit procedure.')


if __name__ == '__main__':
    _test()
