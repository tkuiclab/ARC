#! /usr/bin/env python

import arm_task_rel

rospy.init_node('enable_left_arm', anonymous=True)

rospy.sleep(0.5)

left_arm = = arm_task_rel.ArmTask('/left_arm')

