#!/usr/bin/env python

"""Use to generate arm task and run."""

import sys
from math import radians, degrees, sin, cos
from numpy import multiply

import rospy
import tf
import object_distribution

from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse
from manipulator_h_base_module_msgs.srv import GetJointPose, GetJointPoseResponse

_POS = (.2, 0, .3)  # x, y, z
_ORI = (-70, 0, 0)  # pitch, roll, yaw


class ArmTask:
    """Running arm task class."""

    def __init__(self, _name = '/robotis'):
        """Inital object."""
        self.name = _name
        self.init()
        self.warn_roll = 160
        
    def init(self):
        
        self.__set_pubSub()
        #rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__set_vel_pub.publish(10)
        self.__ik_fail =False


    def __set_pubSub(self):
        print "[Arm] name space : " + str(self.name) 
        self.__set_mode_pub = rospy.Publisher(
            str(self.name) + '/base/set_mode_msg',
            String,
            # latch=True,
            queue_size=1
        )
        self.__joint_pub = rospy.Publisher(
            str(self.name) + '/base/Joint_Control',
            JointPose,
            # latch=True,
            queue_size=1
        )
        self.__ptp_pub = rospy.Publisher(
            str(self.name) + '/base/JointP2P_msg',
            IK_Cmd,
            # latch=True,
            queue_size=1
        )
        self.__cmd_pub = rospy.Publisher(
            str(self.name) + '/base/TaskP2P_msg',
            IK_Cmd,
            # latch=True,
            queue_size=1
        )
        self.__set_vel_pub = rospy.Publisher(
            str(self.name) + '/base/set_velocity',
            Float64,
            latch=True,
            queue_size=1
        )
        self.__status_sub = rospy.Subscriber(
            str(self.name) + '/status',
            StatusMsg,
            self.__status_callback,
            queue_size=1
        )
        # Waiting for topic enable
        rospy.sleep(0.3)

    def __status_callback(self, msg):
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.__ik_fail = True
            #self.stop_task()


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

    @property
    def is_ikfail(self):
        return self.__ik_fail

    def set_speed(self,i_speed):
        self.__set_vel_pub.publish(i_speed)

    def pub_ikCmd(self, mode='line', pos=_POS, euler=_ORI, fai=0):
        """Publish msg of ik cmd (deg) to manager node."""
        # pub_ikCmd('ptp', (x, y , z), (pitch, roll, yaw) )
        
        self.__is_busy = True

        cmd = []

        for p in pos:
            cmd.append(p)
        for e in euler:
            cmd.append(e)
        cmd.append(fai)

        #rospy.loginfo('Sent:{}'.format(cmd))

        if mode == 'line':
            self.__cmd_pub.publish(cmd)
        elif mode == 'ptp':
            self.__ptp_pub.publish(cmd)
        
        
    def stop_task(self):
        """Stop task running."""
        self.__set_mode_pub.publish('')

    def get_fb(self):
        rospy.wait_for_service(self.name + '/base/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                self.name + '/base/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_J7_fb(self):
        rospy.wait_for_service(self.name + '/base/get_joint_pose')
        try:
            get_joint_ang = rospy.ServiceProxy(
                self.name + '/base/get_joint_pose',
                GetJointPose
            )
            res = get_joint_ang(['joint7'])

            j7 = res.joint_value[0] 
            return j7
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

    def relative_move_suction(self, mode='ptp', suction_angle=0, dis=0, blocking = False):
        """Get euler angle and run task."""
        # note:suction_anfle type is degree,  dis is m
        while self.__is_busy and blocking:
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

        fai = degrees(fb.group_redundancy)

        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
            (
                degrees(euler[1]),              
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            ),fai
        )
        

        while self.__is_busy and blocking:
            rospy.sleep(.1)
            #print 'self.__is_busy = ' + str(self.__is_busy)

    def Get_Collision_Avoidance_Cmd(self, desire_cmd, LM1, LM2, SuctionAngle, BinID):
        #desire_cmd = [x, y, z, pitch(deg), roll(deg), yaw(deg)]
        BinArr          = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J']
        TCP_BoundPos_ID = ['A', 'B', 'C', 'D', 'E', 'F']
        BoundPos_Arr = []

        # Define parameters
        LM1_Orig = 64000
        LM2_Orig = 60000
        Arm_Orig = [0.3, -0.12, 0]

        # Calculate the coordinate of whole robot(Arm+LM)
        LM1_Coor = LM1_Orig - LM1
        LM2_Coor = LM2_Orig - LM2
        print 'LM1_Coor = ' + str(LM1_Coor)
        print 'LM2_Coor = ' + str(LM2_Coor)
        Arm_Coor = [desire_cmd[0], Arm_Orig[1] + desire_cmd[1], Arm_Orig[2] + desire_cmd[2]]

        Sys_Coor = [desire_cmd[0],  Arm_Coor[1] + LM2_Coor,  Arm_Coor[2] + LM1_Coor, 
                    desire_cmd[3],  desire_cmd[4],           desire_cmd[5]]
        
        # Check is there any TCP_BoundPos is collision
        fix_vec = [0, 0, 0]
        extract_dis = 0.05

        Binlimit = object_distribution.parse_shelf(id = BinID)                  # get the target bin's limit info
        print '===== Bin Limit Info of ' + str(BinID) + ' ====='
        print 'Binlimit.min_y = ' + str(Binlimit.min_y)
        print 'Binlimit.max_y = ' + str(Binlimit.max_y)
        print 'Binlimit.min_z = ' + str(Binlimit.min_z)
        print 'Binlimit.max_z = ' + str(Binlimit.max_z)
        print '\n'

        for BP_id in TCP_BoundPos_ID:
            BoundPos = self.Get_OneBoundPos_of_TCP(Sys_Coor, SuctionAngle, BP_id)      # get each bound pos of tcp
            # print 'Sys_Coor = ' + str(Sys_Coor)
            print 'BoundPos = ' + str(BoundPos)
            if(BoundPos[0] >= 0.4):
                if (BoundPos[1] > Binlimit.min_y) and (BoundPos[1] < Binlimit.max_y) and (BoundPos[2] > Binlimit.min_z) and (BoundPos[2] < Binlimit.max_z):
                   print 'OKOK\n'
                   pass
                else:
                    print 'fix'
                    if BoundPos[1] <= Binlimit.min_y:
                        tmp = BoundPos[1]
                        BoundPos[1] = Binlimit.min_y + extract_dis ;    print 'BoundPos[1] <= Binlimit.min_y'
                        Sys_Coor[1] = Sys_Coor[1] + abs(BoundPos[1] - tmp)

                    if BoundPos[1] >= Binlimit.max_y: 
                        tmp = BoundPos[1]
                        BoundPos[1] = Binlimit.max_y - extract_dis ;    print 'BoundPos[1] >= Binlimit.max_y'
                        Sys_Coor[1] = Sys_Coor[1] - abs(BoundPos[1] - tmp)

                    if BoundPos[2] <= Binlimit.min_z:
                        tmp = BoundPos[2]
                        BoundPos[2] = Binlimit.min_z + extract_dis ;    print 'BoundPos[2] <= Binlimit.min_z'
                        Sys_Coor[2] = Sys_Coor[2] + abs(BoundPos[2] - tmp)

                    if BoundPos[2] >= Binlimit.max_z: 
                        tmp = BoundPos[2]
                        BoundPos[2] = Binlimit.max_z - extract_dis ;    print 'BoundPos[2] >= Binlimit.max_z'
                        Sys_Coor[2] = Sys_Coor[2] - abs(BoundPos[2] - tmp)
                    print '==\n'
            else:
                pass
        # print '---------------------------------------------------------'
        print 'Final Sys_Coor = ' + str(Sys_Coor)
        a = (80000-LM1)/100000.0
        b = (60000-LM2)/100000.0
        print a
        Fix_output = [Sys_Coor[0],  Sys_Coor[1]-b*0,  Sys_Coor[2]-a*0]
        print 'Fix_output = ' + str(Fix_output)

    def Get_OneBoundPos_of_TCP(self, desire_cmd, angle, BoundID):
        # Get Curr FeedBack
        print 'BoundID = ' + str(BoundID)
        pi = 3.14159
        fb = self.get_fb()
        pos = fb.group_pose.position

        # Get desire cmd
        pos.x = desire_cmd[0]
        pos.y = desire_cmd[1]
        pos.z = desire_cmd[2]
        euler = [ desire_cmd[3], desire_cmd[4], desire_cmd[5] ]

        # Get n s a vector
        rot = self.nsa2rotation(euler)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)

        # init var(m)
        len_f  = 0.08
        len_TM = 0.02
        T_U    = 0.06
        T_D    = 0.035
        Suct_L = 0.03
        Suct_R = 0.05
        len_tool = 0.15

        offset = [0,0,0]
        tmp_pos  = [0,0,0]
        BoundPos = [0,0,0]

        # Get_OneBoundPos_of_TCP 
        if(BoundID=='A'):  # Pipe_FL(A)
            tmp_pos[0] = pos.x + len_f*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_f*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, -T_U, -Suct_L)

        elif(BoundID=='B'):
            tmp_pos[0] = pos.x + len_f*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_f*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, -T_U, Suct_R)

        elif(BoundID=='C'):
            tmp_pos[0] = pos.x + len_f*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_f*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, T_D, -Suct_L)

        elif(BoundID=='D'):
            tmp_pos[0] = pos.x + len_f*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_f*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, T_D, Suct_R)

        elif(BoundID=='E'):
            tmp_pos[0] = pos.x + len_TM*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_TM*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, T_D, -Suct_L)

        elif(BoundID=='F'):
            tmp_pos[0] = pos.x + len_TM*cos(angle) + len_tool
            tmp_pos[1] = pos.y
            tmp_pos[2] = pos.z + len_TM*sin(angle)
            offset   = self.Get_Suction_rel_offset(desire_cmd, angle, T_D,  Suct_R)
        
        else :
            print 'err BinID\n'
        
        BoundPos[0] = tmp_pos[0] + offset[0]
        BoundPos[1] = tmp_pos[1] + offset[1]
        BoundPos[2] = tmp_pos[2] + offset[2]
        return BoundPos
        
    def Get_Suction_rel_offset(self, desire_cmd, angle, dis, vec_s_len):
        pi = 3.14159
        # Get desire cmd (cal offset only need euler)
        euler = [ desire_cmd[3]*pi/180, desire_cmd[4]*pi/180, desire_cmd[5]*pi/180 ]

        # Get n s a unit vector
        rot = self.nsa2rotation(euler)
        vec_s, vec_n, vec_a = self.rotation2vector(rot)

        # calculate n s a len
        rate_n = float((90-angle)/90.0)
        rate_a = float(angle/90.0)
        n = rate_n*(dis)
        a = rate_a*(dis)
        s = vec_s_len

        offset  = [0, 0, 0]
        offset += multiply(vec_n, n)
        offset += multiply(vec_s, s)
        offset += multiply(vec_a, a)
        
        return offset

    def relative_move_nsa(self, mode='ptp', n=0, s=0, a=0, blocking = False):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        while self.__is_busy and blocking:
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
    
        fai=0
        self.pub_ikCmd(
            mode,
            (pos.x + move[1], pos.y + move[0], pos.z + move[2]),
            (
                degrees(euler[1]),              
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            ),fai
        )

        while self.__is_busy and blocking:
            rospy.sleep(.1)
    
    def get_fai(self, desire_roll):
        J7_FB    = self.get_J7_fb()
        fb       = self.get_fb()
        ori      = fb.group_pose.orientation
        curr_fai = fb.group_redundancy
        euler    = self.quaternion2euler(ori)

        curr_j7   = degrees(J7_FB)
        curr_roll = degrees(euler[2]+radians(90))

        # desire_j7 = curr_j7 + (curr_roll - desire_roll) + 10
        desire_j7 = desire_roll 

        fai = 0
        print 'desire_roll = ' + str(desire_roll)
        if abs(desire_roll) >= self.warn_roll:
            #print 'abs(desire_roll >= self.warn_roll)\n'
            if (curr_j7 > 0 and curr_j7 <= 180):
                print '1'
                if desire_j7 > -180 and desire_j7 < 0:
                    print '2'
                    fai = -20
                else:
                    fai = curr_fai

            elif curr_j7 >= -180 and curr_j7 < 0:
                print '3'
                if desire_j7 > 0 and desire_j7 < 180:
                    print '4'
                    fai = 20
                else:
                    fai = curr_fai

            else:
                fai = curr_fai
        else:
            fai = curr_fai

        # print '===============================\n'
        # print 'desire_roll = ' + str(desire_roll)
        # print 'curr_j7 = ' + str(curr_j7)
        # print 'desire_j7 = ' + str(desire_j7)
        # print 'send fai = ' + str(fai)
        # print '===============================\n'
        return fai
                

        # future_j7 = curr_j7 + (curr_roll - desire_roll)

    def move_2_Abs_Roll(self, abs_roll, blocking = False):
        while self.__is_busy and blocking:
            rospy.sleep(.1)
        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        abs_roll = abs_roll*3.14/180.0

        tmp_roll = degrees(abs_roll)
        self.pub_ikCmd(
            'ptp',
            (pos.x, pos.y, pos.z),  
            (
                degrees(euler[1]),              
                degrees(abs_roll),
                degrees(euler[0])
            ),self.get_fai(tmp_roll)
        )

        while self.__is_busy and blocking:
            rospy.sleep(.1)


    def relative_move_nsa_rot_pry(self, mode='ptp', n=0, s=0, a=0, yaw=0, pitch=0, roll=0, blocking = False):
        """Get euler angle and run task."""
        # ============================================================================
        # note1: for nsa rotation only
        # Note2: Although the fn will complete the motion simultaneously, 
        #        however, in ik cmd, it will first move along with nsa, then rot pry
        # ============================================================================

        while self.__is_busy and blocking:
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

        while self.__is_busy and blocking:
            rospy.sleep(.1)

    def relative_move_xyz_rot_pry(self, mode='ptp', x=0, y=0, z=0, yaw=0, pitch=0, roll=0, fai=0, blocking=False):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        # euler[0~2] = [r p y] = [a s n]
        while self.__is_busy and blocking:
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
            ),fai
        )

        while self.__is_busy and blocking:
            rospy.sleep(.1)

    def relative_rot_nsa(self, mode='ptp', pitch=0, roll=0, yaw=0, exe=True, blocking=False):
        """Get euler angle and run task."""
        # note:for nsa rotation only
        # euler[0~2] = [r p y] = [a s n]
        
        tmp_ori = [yaw, pitch, roll]
        # print '\n2.relative_rot_nsa = ' + str(pitch)
        # print '===[n, s, a] = ' + str(tmp_ori)
        while self.__is_busy and blocking:
            rospy.sleep(.1)

        fb    = self.get_fb()
        pos   = fb.group_pose.position
        ori   = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)

        if pitch!=0 and abs(euler[0]) > 0.0001:
            pitch = 0
            print'err, yaw(n) is not equal to 0, pitch(s) cannot do relative motion'
            # return 

        # Avoid J7_over 180
        tmp_roll = degrees(euler[2] + radians(roll+90) )
        if exe == True:
            self.pub_ikCmd(
                mode,
                (pos.x, pos.y, pos.z),
                (
                    degrees(euler[1] + radians(pitch) ),
                    degrees(euler[2] + radians(roll+90) ),
                    degrees(euler[0] + radians(yaw) )
                ),self.get_fai(tmp_roll)
            )
            while self.__is_busy and blocking:
                rospy.sleep(.1)
        else:
            print '[relative_rot_nsa Error] exe = false'
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

        # Avoid J7_Over 180
        fai=0
        self.pub_ikCmd(
            mode,
            (rel_pos[0], rel_pos[1], rel_pos[2]),
            (
                degrees(rel_pry[1]), 
                degrees(rel_pry[2])+90, 
                degrees(rel_pry[0]), 
             ),fai
        )
        

        while self.__is_busy:
            rospy.sleep(.1)
        # ==============

    def relative_xyz_base(self, mode='ptp', x=0, y=0, z=0, fai=0, blocking = False):
        """relative move xyz with manipulator base axis."""
        while self.__is_busy and blocking:
            # print 'first self.__is_busy = ' + str(self.__is_busy)
            rospy.sleep(.1)

        fb = self.get_fb()
        pos = fb.group_pose.position
        ori = fb.group_pose.orientation
        euler = self.quaternion2euler(ori)
        
        # # Avoid J7 Over 180
        # for nsa
        tmp_roll = degrees(euler[2]+(90*3.14156/180))
        
        self.pub_ikCmd(
            mode,
            (pos.x + x, pos.y + y, pos.z + z),
            (
                degrees(euler[1]),
                degrees(euler[2]+(90*3.14156/180)),
                degrees(euler[0])
            ),self.get_fai(tmp_roll)
        )


        print 'befoer second busy self.__is_busy = ' + str(self.__is_busy)
        while self.__is_busy and blocking:
            # print 'self.__is_busy = ' + str(self.__is_busy)
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
    