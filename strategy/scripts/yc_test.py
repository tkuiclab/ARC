#! /usr/bin/env python

import sys
import roslaunch
import roslib

import rospkg
import rospy
from math import radians, degrees, sin, cos, tan, pi
from numpy import multiply
import numpy
import object_distribution

def show_bin_data(bin):
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)
    print 'bin.block = ' + str(bin.block)

if __name__ == '__main__':
    rospy.init_node('yc_test', disable_signals=True)

    try:
        bin_a = object_distribution.BinInfo()
        print 'start'
        a = object_distribution.parse_shelf(3)

        print 'Bin id is ' + str(a.block)
        print 'min_y = ' + str(a.min_y)
        print 'max_y = ' + str(a.max_y)
        print 'min_z = ' + str(a.min_z)
        print 'max_z = ' + str(a.max_z)




        rospy.spin()
    except rospy.ROSInterruptException:
        pass
