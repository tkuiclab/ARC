#!/usr/bin/env python

"""Convert the labels."""

from __future__ import print_function
import rospy
from labels import *


_labelCvt = dict(zip(amazonLabelsList, ourLabelsList))

def offical2Our(key):
    """Convert offical label to our defined label (str)."""
    try:
        return _labelCvt[key]
    except KeyError as e:
        rospy.logwarn("The labels don't have the name! {}".format(e))
        return None

def our2Offical(val):
    """Convert our defined label to offical label (str)."""
    return _labelCvt.keys()[_labelCvt.values().index(val)]
