#!/usr/bin/env python

"""Strategy for picking task."""

from __future__ import print_function
import rospy
import actionlib

import obj_pose.msg
from strategy.srv import *
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Char, Float64, String, ByteMultiArray
from actionlib_msgs.msg import GoalID, GoalStatusArray

import LM_Control
import arm_task_rel
from config import *
from gripper import *
from task_parser import *


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

PhotoPose = 26
VisionProcess = 27
Rotate2PickPose = 28
WaitVision = 29
CheckIsHold = 30

_REL_GO_BIN_FRONT = .1
_BIN_MAX_DOWN = .135
_CAM_2_TOOL_Z = .13
_CAM_2_TOOL_Y = .08

# For checking vacuum function
check_next_states = [
    RobotMove2Box,
    Prepare2Place,
    Down2Place
]


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
        self.obj_pose = result.object_pose
        if not result.success:
            rospy.logwarn('ROI Fail!! obj -> {}'.format(self.now_pick.item))
            self.state = FinishTask
        else:
            self.obj_pose = result.object_pose
            print('obj_pose {}'.format(self.obj_pose))
            self.state = self.next_state

    def ifsuck_cb(self, res):
        """suction status callback."""
        self.suck_ary = res.data

    @property
    def suck_num(self):
        return sum(self.suck_ary)

    @property
    def finish(self):
        return self.state == WaitTask

    def save_item_location(self, item_location):
        """Load item location file."""
        rospy.loginfo("[Pick] Save item_location")
        self.item_location = item_location
        self.parse_json_2_pick_list()

    def save_order(self, order):
        """Load order file."""
        rospy.loginfo("[Pick] Save order")
        self.order = order
        self.parse_json_2_pick_list()

    def parse_json_2_pick_list(self):
        if self.item_location is None or self.order is None:
            return
        rospy.loginfo(
            "[Pick] Parse JSON of item_location and order to pick_list")
        self.pick_list = make_pick_list(self.item_location, self.order)

    def is_ready(self):
        return self.pick_list is not None

    def pick_get_one(self):
        """Get one pick task."""
        """Set: self.Bin, self.pick_id, self.Box"""
        if self.pick_id >= len(self.pick_list):
            return False

        self.now_pick = self.pick_list[self.pick_id]
        self.bin = self.now_pick.from_bin.lower()
        self.item = self.now_pick.item

        if self.now_pick.to_box == 'A1':
            self.box = 'a'
        elif self.now_pick.to_box == '1A5':
            self.box = 'b'
        elif self.now_pick.to_box == '1B2':
            self.box = 'c'
        else:
            rospy.logerr('Error pick.to_box = {}'.format(self.now_pick.to_box))

        self.pick_id += 1
        return True

    def run(self):
        self.pick_id = 0
        self.pick_get_one()
        self.state = RobotMove2Bin

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
            self.next_state = VisionProcess

            # Move to bin using linear mobile
            self.Is_BaseShiftOK = False
            self.LM.pub_LM_Cmd(2, GetShift('Bin', 'x', self.bin))
            rospy.sleep(0.3)
            self.LM.pub_LM_Cmd(1, GetShift('Bin', 'z', self.bin))
            # Arm move to initial pose
            gripper_suction_up()
            self.Arm.pub_ikCmd('ptp', (0.3, 0.0, 0.3), (-90, 0, 0))

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
            self.Arm.pub_ikCmd('ptp', (0.3, 0.0, 0.4), (-105, 0, 0))

        # Vision processing
        elif self.state == VisionProcess:
            self.info = "(Catch) Request Vision Process: {}".format(self.now_pick.item)
            print(self.info)

            # ActionLib request
            goal = obj_pose.msg.ObjectPoseGoal(object_name = self.now_pick.item)
            self.__obj_pose_client.send_goal(
                goal,
                feedback_cb=self.obj_pose_feedback_cb,
                done_cb=self.obj_pose_done_cb
            )

            # Change state
            self.state = WaitVision
            self.next_state = Move2BinFront

        # Arm move to bin front
        elif self.state == Move2BinFront:
            self.info = "(Catch) Arm Move to Bin Front {}".format(self.bin)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Rotate2PickPose

            # cam_z = self.obj_pose.linear.z - 0.13
            self.Arm.relative_move_nsa(a=_REL_GO_BIN_FRONT)

        # Tune pose to fit object
        elif self.state == Rotate2PickPose:
            self.info = "(Catch) Arm Rotate 2 Pick Pose"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = PickObj

            tool_x = self.obj_pose.linear.x
            tool_y = self.obj_pose.linear.y + _CAM_2_TOOL_Y
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

            tool_z = self.obj_pose.linear.z - _CAM_2_TOOL_Z - _REL_GO_BIN_FRONT
            self.Arm.relative_move_nsa(a=tool_z)

        # Move upper
        elif self.state == Up2LeaveBin:
            self.info = "(Catch) Arm Up"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = LeaveBin

            self.Arm.relative_move_nsa(n=-0.05)

        # Move out of bin
        elif self.state == LeaveBin:
            self.info = "(Catch) Arm Leave Bin"
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = RobotMove2Box

            # self.Arm.relative_move_nsa(a=-0.05)
            tool_z = self.obj_pose.linear.z - _CAM_2_TOOL_Z
            self.Arm.relative_move_nsa(a=-tool_z)

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
            self.Arm.pub_ikCmd('ptp', (0.3, 0, 0.3), (-90, 0, 0))

        # Prepare to place
        elif self.state == Prepare2Place:
            self.info = "(GoBox) Arm Prepare to Place {}".format(self.box)
            print(self.info)

            # Change state
            self.state = WaitRobot
            self.next_state = Down2Place

            self.Arm.pub_ikCmd('ptp', (0.35, 0, 0.2), (-180, 0, 0))

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
            self.next_state = FinishTask

            gripper_vaccum_off()
            self.update_location_file()

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
            if not self.Last_LMArrive and self.Is_LMArrive:
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
            # Can get next task from pick list
            if self.pick_get_one():
                self.Is_BaseShiftOK = False
                self.state = RobotMove2Bin
                self.info = "Finish Pick One Object"
            else:
                self.task_finish()
                self.info = "Finish Pick All of Task"
            print(self.info)

    def check_vaccum_by_next_state(self, n_s):
        """Checking status of vacuum is hold."""
        # These state need to check vacuum is hold
        if n_s in check_next_states:
            self.info = "(Check) Status of Suction: {}".format(self.suck_num)
            print(self.info)
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
        write_pick_task_location(self.item_loc)

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
        self.pick_list = None
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
        self.state = WaitTask
        self.next_state = WaitTask
        self.Is_ArmBusy = False
        self.Is_LMBusy = False
        self.Last_LM_Busy = False
        self.Is_LMArrive = True
        self.Last_LMArrive = True
        self.Is_BaseShiftOK = False


def _test():
    """Testing this module."""
    rospy.init_node('picking_strategy', anonymous=True, disable_signals=True)
    try:
        arm = arm_task_rel.ArmTask()
        lm = LM_Control.CLM_Control()
        s = PickTask(arm, lm)

        # Setting picking list
        s.pick_list, s.item_loc = read_pick_task_and_location()
        for info in s.pick_list:
            print("item:", info.item, "from_bin:", info.from_bin, "to_box:", info.to_box)

        rospy.sleep(0.3)
        s.run()

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown() and not s.finish:
            s.pick_core()
            rate.sleep()
    except KeyboardInterrupt as e:
        print('KeyboardInterrupt exit procedure.')


if __name__ == '__main__':
    _test()
