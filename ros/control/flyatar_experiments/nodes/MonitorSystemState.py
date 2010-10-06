#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import InBounds,FlyView

class InBoundsSubscriber:
    def __init__(self):
        self.in_bounds_sub = rospy.Subscriber('InBounds',InBounds,self.in_bounds_callback)
        self.initialized = False

    def in_bounds_callback(self,data):
        self.bounds_radius = data.bounds_radius
        self.robot_in_bounds = data.robot_in_bounds
        self.fly_in_bounds = data.fly_in_bounds
        if not self.initialized:
            self.initialized = True

class FlyViewSubscriber:
    def __init__(self):
        self.fly_view_sub = rospy.Subscriber("FlyView",FlyView,self.fly_view_callback)
        self.initialized = False

    def fly_view_callback(self,data):
        self.robot_position_x = data.robot_position_x
        self.robot_position_y = data.robot_position_y
        self.robot_angle = data.robot_angle
        self.robot_distance = data.robot_distance
        self.robot_in_front_of_fly = data.robot_in_front_of_fly

        if not self.initialized:
            self.initialized = True
