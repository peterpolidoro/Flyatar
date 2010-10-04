#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import InBounds

class InBoundsSubscriber:
    def __init__(self):
        self.in_bounds_sub = rospy.Subscriber('InBounds',InBounds,self.in_bounds_callback)
        self.bounds_radius = None
        self.robot_in_bounds = None
        self.fly_in_bounds = None

    def in_bounds_callback(self,data):
        self.bounds_radius = data.bounds_radius
        self.robot_in_bounds = data.robot_in_bounds
        self.fly_in_bounds = data.fly_in_bounds

