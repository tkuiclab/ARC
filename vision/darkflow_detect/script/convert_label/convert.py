#!/usr/bin/env python

"""Convert the labels."""

from __future__ import print_function
from labels import *


_labelCvt = dict(zip(amazonLabelsList, ourLabelsList))

def offical2Our(key):
    """Convert offical label to our defined label (str)."""
    return _labelCvt[key]

def our2Offical(val):
    """Convert our defined label to offical label (str)."""
    return _labelCvt.keys()[_labelCvt.values().index(val)]
