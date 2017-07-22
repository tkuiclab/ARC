#!/usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos
from numpy import multiply

import rospy
import tf

from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse

_POS = (.2, 0, .3)  # x, y, z
_ORI = (-70, 0, 0)  # pitch, roll, yaw


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        #rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__set_vel_pub.publish(20)

    def __set_pubSub(self):
        self.__set_mode_pub = rospy.Publisher(
            '/robotis/base/set_mode_msg',
            String,
            # latch=True,
            queue_size=1
        )
        self.__joint_pub = rospy.Publisher(
            '/robotis/base/Joint_Control',
            JointPose,
            # latch=True,
            queue_size=1
        )
        self.__ptp_pub = rospy.Publisher(
            '/robotis/base/JointP2P_msg',
            IK_Cmd,
            # latch=True,
            queue_size=1
        )
        self.__cmd_pub = rospy.Publisher(
            '/robotis/base/TaskP2P_msg',
            IK_Cmd,
            # latch=True,
            queue_size=1
        )
        self.__set_vel_pub = rospy.Publisher(
            '/robotis/base/set_velocity',
            Float64,
            latch=True,
            queue_size=1
        )
        self.__status_sub = rospy.Subscriber(
            '/robotis/status',
            StatusMsg,
            self.__status_callback,
            queue_size=1
        )
        # Waiting for topic enable
        rospy.sleep(0.3)

    def __status_callback(self, msg):
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.stop_task()

        elif 'End Trajectory' in msg.status_msg:
            self.__is_busy = False

    def pub_jointCmd(self, cmd=[0, 0, 0, 0, 0, 0, 0]):
        """Publish msg of joint cmd (rad) to manager node."""
        name  = list()
        value = list()         
        for i, val in enumerate(cmd):
            name.append('joint{}'.format(i+1))
            value.append(val)

        self.__joint_pub.publish(JointPose(name, value))
        self.__is_busy = True

    def home(self):
        self.pub_jointCmd([0,0,0,0, 0,0,0])

    def pub_ikCmd(self, mode='line', pos=_POS, euler=_ORI):
        """Publish msg of ik cmd (deg) to manager node."""
        # pub_ikCmd('ptp', (x, y , z), (pitch, roll, yaw) )
        # while self.__is_busy:
        #     rospy.sleep(.1)

        self.__is_busy = True

        cmd = []

        for p in pos:
            cmd.append(p)
        for e in euler:
            cmd.append(e)

        rospy.loginfo('Sent:{}'.format(cmd))

        if mode == 'line':
            self.__cmd_pub.publish(cmd)
        elif mode == 'ptp':
            self.__ptp_pub.publish(cmd)

        
    def stop_task(self):
        """Stop task running."""
        self.__set_mode_pub.publish('')

    def get_fb(self):
        rospy.wait_for_service('/robotis/base/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                '/robotis/base/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def quaternion2euler(self, ori):
        quaternion = (
            ori.x,
            ori.y,
            ori.z,
            ori.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return (pitch, roll, yaw)

    def euler2rotation(self, euler):  # jmp_rot
        Cx = cos(euler[1])   # 1 0 2     
        Sx = sin(euler[1])
        Cy = cos(euler[0])
        Sy = sin(euler[0])
        Cz = cos(euler[2])
        Sz = sin(euler[2])

        return [
            [Cz * Sy + Sz * Sx * Cy,   Cz * Cy - Sz * Sx * Sy,  -Sz * Cx],
            [Sz * Sy - Cz * Sx * Cy,   Sz * Cy + Cz * Sx * Sy,   Cz * Cx],
            [               Cx * Cy,                 -Cx * Sy,        Sx]
        ]

    def nsa2rotation(self, euler):  # jmp_rot
        # print 'euler = ' + str(euler[1]*180/3.1415926)+ ', ' + str(euler[0]*180/3.1415926)+ ', ' + str(euler[2]*180/3.1415926)
        Cx = cos(euler[1])   # 1 0 2 rpy     
        Sx = sin(euler[1])
        
        Cy = cos(euler[0]+(0*3.14156/180))
        Sy = sin(euler[0]+(0*3.14156/180))
        Cz = cos(euler[2]+(90*3.1416/180))
        Sz = sin(euler[2]+(90*3.1416/180))

        return [
                [Cy*Cz              , -Cy*Sz            ,     Sy],
                [Sx*Sy*Cz + Cx*Sz   , -Sx*Sy*Sz + Cx*Cz , -Sx*Cy],
                [-Cx*Sy*Cz + Sx*Sz  , Cx*Sy*Sz + Sx*Cz  ,  Cx*Cy]
        ]

    def rotation2vector(self, rot):
        vec_n = [rot[0][0], rot[1][0], rot[2][0]]
        vec_s = [rot[0][1], rot[1][1], rot[2][1]]
        vec_a = [rot[0][2], rot[1][2], rot[2][2]]
        return vec_n, vec_s, vec_a   

    def relative_move_suction(self, mode='ptp', suction_angle=0, dis=0 ):
        """Get euler angle and run task."""
        # note:suction_anfle type is degree,  dis is m
        while self.__is_busy:
            rospy.sleep(.1)

        # ======= Calculate suction vector start ========
        rate_n = float((90-suction_angle)/90.0)
        rate_a = float(suction_angle/90.0)
        n = rate_n*dis
        a = rate_a*dis
        s = 0
        # print 'dis = ' + str(dis)
        # print 'rate_n = ' + str(rate_n)
        # print 'rate_a = ' + str(rate_a)
        # print 'n = ' + str(n)
        # print 'a = ' + str(a)
        # ======= Calculate suction vector over  ========

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        rot = self.nsa2rotation(euler)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)
        
        move = [0, 0, 0]

        if n != 0:
            move += multiply(vec_n, n)
        if s != 0:
            move += multiply(vec_s, s)
        if a != 0:
            move += multiply(vec_a, a)
    
        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
            (
                degrees(euler[1]),              
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    def relative_move_nsa(self, mode='ptp', n=0, s=0, a=0):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        while self.__is_busy:
            rospy.sleep(.1)

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        rot = self.nsa2rotation(euler)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)
        
        move = [0, 0, 0]

        if n != 0:
            move += multiply(vec_n, n)
        if s != 0:
            move += multiply(vec_s, s)
        if a != 0:
            move += multiply(vec_a, a)
    
        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
            (
                degrees(euler[1]),              
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    def relative_move_nsa_rot_pry(self, mode='ptp', n=0, s=0, a=0, yaw=0, pitch=0, roll=0):
        """Get euler angle and run task."""
        # ============================================================================
        # note1: for nsa rotation only
        # Note2: Although the fn will complete the motion simultaneously, 
        #        however, in ik cmd, it will first move along with nsa, then rot pry
        # ============================================================================

        while self.__is_busy:
            rospy.sleep(.1)

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        rot = self.nsa2rotation(euler)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)
        
        move = [0, 0, 0]

        if n != 0:
            move += multiply(vec_n, n)
        if s != 0:
            move += multiply(vec_s, s)
        if a != 0:
            move += multiply(vec_a, a)

        if pitch!=0 and abs(euler[0]) > 0.0001:
            pitch = 0
            print'err, yaw(n) is not equal to 0, pitch(s) cannot do relative motion'
        
        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),  #######
            (
                degrees(euler[1]+(pitch*3.14156/180)),              
                degrees(euler[2]+((roll+90)*3.14156/180)),
                degrees(euler[0]+(yaw*3.14156/180))
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)


    def relative_move_xyz_rot_pry(self, mode='ptp', x=0, y=0, z=0, yaw=0, pitch=0, roll=0):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        # euler[0~2] = [r p y] = [a s n]
        while self.__is_busy:
            rospy.sleep(.1)

        fb    = self.get_fb()
        pos   = fb.group_pose.position
        ori   = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        if pitch!=0 and abs(euler[0]) > 0.0001:
            pitch = 0
            print'err, yaw(n) is not equal to 0, pitch(s) cannot do relative motion'
            # return 
        self.pub_ikCmd(
            mode,
            (pos.x + x, pos.y + y, pos.z + z),
            (
                degrees(euler[1]+(pitch*3.14156/180)),              
                degrees(euler[2]+((roll+90)*3.14156/180)),
                degrees(euler[0]+(yaw*3.14156/180))
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    def relative_rot_nsa(self, mode='ptp', pitch=0, roll=0, yaw=0, exe=True):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        # euler[0~2] = [r p y] = [a s n]
        
        tmp_ori = [yaw, pitch, roll]
        print '\n2.relative_rot_nsa = ' + str(pitch)
        print '===[n, s, a] = ' + str(tmp_ori)
        while self.__is_busy:
            rospy.sleep(.1)

        fb    = self.get_fb()
        pos   = fb.group_pose.position
        ori   = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        if pitch!=0 and abs(euler[0]) > 0.0001:
            pitch = 0
            print'err, yaw(n) is not equal to 0, pitch(s) cannot do relative motion'
            # return 

        if exe == True:
            self.pub_ikCmd(
                mode,
                (pos.x, pos.y, pos.z),
                (
                    # degrees(euler[1]+(pitch*3.14156/180)),              
                    # degrees(euler[2]+((roll+90)*3.14156/180)),
                    # degrees(euler[0]+(yaw*3.14156/180))
                    
                    degrees(euler[1] + radians(pitch) ),
                    degrees(euler[2] + radians(roll+90) + radians(0) ),
                    degrees(euler[0] + radians(yaw) )
                )
            )
            while self.__is_busy:
                rospy.sleep(.1)
        else:
            print 'exe = false'
            rel_fb_pitch = euler[1]+(pitch*3.14156/180)
            rel_fb_roll  = euler[2]+((roll+90)*3.14156/180)
            rel_fb_yaw   = euler[0]+(yaw*3.14156/180)
            rel_xyz_pry = [pos.x, pos.y, pos.z, rel_fb_pitch, rel_fb_roll, rel_fb_yaw]
            
            print 'rel_fb_pitch = ' + str(rel_fb_pitch*180/3.1416)
            print 'rel_fb_roll = '  + str(rel_fb_roll*180/3.1416)
            print 'rel_fb_yaw = '   + str(rel_fb_yaw*180/3.1416)
            return rel_xyz_pry

    def relative_rot_pry_move_nsa(self, mode='ptp', n=0, s=0, a=0, yaw=0, pitch=0, roll=0):
        # ============================================================================
        # note1: for nsa rotation only
        # Note2: Although the fn will complete the motion simultaneously, 
        #        however, in ik cmd, it will first rot pry, then move along with nsa
        # ============================================================================
        while self.__is_busy:
            rospy.sleep(.1)

        # [ ========== rel rot ==========]
        fb    = self.get_fb()
        pos   = fb.group_pose.position
        ori   = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        orig_pitch = degrees(euler[1])
        orig_roll  = degrees(euler[2] + radians(roll+90) )
        orig_yaw   = degrees(euler[0])

        orig_pry     = [orig_pitch, orig_roll,  orig_yaw]
        rel_xyz_pry  = [pos.x, pos.y, pos.z, pitch, roll, yaw]
        rel_pry      = [ orig_pry[0]+rel_xyz_pry[3],  orig_pry[1]+rel_xyz_pry[4],  orig_pry[2]+rel_xyz_pry[5] ]


        # [ ========== rel move ==========]
        rel_pry = [radians(rel_pry[2]),  radians(rel_pry[0]),  radians(rel_pry[1]-90)]
        rot     = self.nsa2rotation(rel_pry)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)
        
        move = [0, 0, 0]

        if n != 0:
            move += multiply(vec_n, n)
        if s != 0:
            move += multiply(vec_s, s)
        if a != 0:
            move += multiply(vec_a, a)
        
        rel_pos = [pos.x + move[1],  pos.y + move[0],  pos.z + move[2],]

        self.pub_ikCmd(
            mode,
            (rel_pos[0], rel_pos[1], rel_pos[2]),
            (
                degrees(rel_pry[1]), 
                degrees(rel_pry[2])+90, 
                degrees(rel_pry[0]), 
             )
        )
        

        while self.__is_busy:
            rospy.sleep(.1)
        # ==============

    def relative_control(self, mode='ptp', n=0, s=0, a=0):
        # """Get euler angle and run task."""
        # # note:for ICLab rotation only
        while self.__is_busy:
            rospy.sleep(.1)

        # fb = self.get_fb()
        # pos = fb.group_pose.position
        # ori = fb.group_pose.orientation
        # euler = self.quaternion2euler(ori)
        # rot = self.euler2rotation(euler)
        # vec_n, vec_s, vec_a = self.rotation2vector(rot)
        
        # move = [0, 0, 0]
        # if n != 0:
        #     move += multiply(vec_n, n)
        # if s != 0:
        #     move += multiply(vec_s, s)
        # if a != 0:
        #     move += multiply(vec_a, a)

        # self.pub_ikCmd(
        #     mode,
        #     (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
        #     (
        #         degrees(euler[1]),   # 1 0 2             
        #         degrees(euler[0]),
        #         degrees(euler[2])
        #     )
        # )

        # while self.__is_busy:
        #     rospy.sleep(.1)

    def relative_xyz_base(self, mode='ptp', x=0, y=0, z=0):
        """relative move xyz with manipulator base axis."""
        while self.__is_busy:
            rospy.sleep(.1)

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        # self.pub_ikCmd(
        #     mode,
        #     (pos.x + x, pos.y + y, pos.z + z),
        #     (
        #         degrees(euler[1]),
        #         degrees(euler[0]),
        #         degrees(euler[2])
        #     )
        # )
        
        # for nsa
        self.pub_ikCmd(
            mode,
            (pos.x + x, pos.y + y, pos.z + z),
            (
                degrees(euler[1]),
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    def Move2_Abs_xyz(self, x, y, z, mode='ptp'):
        """relative move xyz with manipulator base axis."""
        while self.__is_busy:
            rospy.sleep(.1)

        fb = self.get_fb()
        # pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        
        # for nsa
        self.pub_ikCmd(
            mode,
            (x, y, z),
            (
                degrees(euler[1]),
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            )
        )

        while self.__is_busy:
            rospy.sleep(.1)

    @property
    def busy(self):
        return self.__is_busy


if __name__ == '__main__':

    rospy.init_node('robot_arm_task', anonymous=True)
    rospy.loginfo('robot arm task running')

    task = ArmTask()
    rospy.sleep(0.3)

    # # task.pub_ikCmd('ptp')
    # task.pub_ikCmd('ptp', (0.30, 0.0 , 0.2), (-95, 0, 0, 0) )
    # rospy.sleep(0.3)

    # task.relative_control(a=.05) #x
    # rospy.sleep(0.3)

    # task.relative_control(a=-0.05) #x
    # rospy.sleep(0.3)

    # task.relative_control(n=-0.05) #x
    # rospy.sleep(0.3)

    # task.relative_control(n=0.05) #x
    # rospy.sleep(0.3)

    # task.relative_control(s=.05) #x
    # rospy.sleep(0.3)

    # task.relative_control(s=-0.05) #x
    # rospy.sleep(0.3)
    