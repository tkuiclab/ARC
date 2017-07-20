import rospy
from vacuum_cmd_msg.srv import VacuumCmd

# 2017/07/19 Gripper Parameter Backup
# cam2tool_y = -0.025
# cam2tool_z = 0.14
# gripper_length = 0.04
# cam2center_y = 0.025
# cam2center_y_4_tote = 0.05

# NOW
cam2tool_z = 0.27 #0.26
gripper_length = 0.04
cam2center_y = 0.035
cam2center_y_4_tote = 0.05

def robot_cmd_client(cmd):
    rospy.wait_for_service('/robot_cmd')
    try:
        client = rospy.ServiceProxy(
            '/robot_cmd',
            VacuumCmd
        )

        client(cmd)
       
    except rospy.ServiceException, e:
        print "Service call (Vacuum) failed: %s" % e

def gripper_vaccum_on():
    robot_cmd_client('vacuumOn')
    print('Vaccum On')
    rospy.sleep(0.5)

def gripper_vaccum_off():
    robot_cmd_client('vacuumOff')
    print('Vaccum Off')
    rospy.sleep(0.3)

def gripper_suction_up():
    robot_cmd_client('suctionUp')
    print('Suction Up')

def gripper_suction_down():
    robot_cmd_client('suctionDown')
    print('Suction Down')

def gripper_suction_deg(deg):
    robot_cmd_client(str(deg))
    print('Suction Move : '+str(deg))
