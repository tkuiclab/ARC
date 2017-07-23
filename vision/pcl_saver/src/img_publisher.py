#!/usr/bin/env python
# Lucas Walter
# GNU GPLv3
# Load an image from disk and publish it repeatedly

import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Image

if __name__ == '__main__': 
    rospy.init_node('img_publisher')

    name = sys.argv[1]
    image = cv2.imread(name)
    #cv2.imshow("im", image)
    #cv2.waitKey(5)

    hz = rospy.get_param("~rate", 10)
    rate = rospy.Rate(hz)



    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)

    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.encoding = 'bgr8'
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.step = image.shape[1] * 3
    msg.data = image.tostring()
    pub.publish(msg)


    rospy.loginfo("Publish " + name + " to Topic : /camera/rgb/image_raw")
    

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()