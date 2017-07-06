import rospy
from vacuum_cmd_msg.srv import VacuumCmd


cam2tool_y = -0.11  #-0.095  #cam axis
cam2tool_z = 0.23   # + 0.035

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
