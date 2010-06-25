#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy

import numpy
# from geometry_msgs.msg import PoseStamped

class Viterbi:
    def __init__(self):
