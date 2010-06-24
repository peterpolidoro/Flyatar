#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy

import numpy
# from geometry_msgs.msg import PoseStamped

class StopWalk:
    def __init__(self):
        self.vel_threshold_low = 1
        self.vel_threshold_high = 2.5
        self.states = {'stopped' : True, 'walking' : False}
        self.state = self.states['stopped']
        self.state_prev = self.state

    def classify(self,vmag):
        if (self.state == self.states['stopped']) and (self.vel_threshold_high < vmag):
            state = self.states['walking']
        elif (self.state == self.states['walking']) and (self.vel_threshold_high < vmag):
            state = self.states['stopped']

        # Must be in same state for at least two frames
        if state == self.state_prev:
            self.state = state

        self.state_prev = state

        return self.state
