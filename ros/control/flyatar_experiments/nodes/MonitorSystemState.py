#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics,InBounds,FlyView

class InBoundsSubscriber:
    def __init__(self):
        self.in_bounds_sub = rospy.Subscriber('InBounds',InBounds,self.in_bounds_callback)
        self.in_bounds = InBounds()
        self.initialized = False

    def in_bounds_callback(self,data):
        self.in_bounds = data
        if not self.initialized:
            self.initialized = True

class FlyViewSubscriber:
    def __init__(self):
        self.fly_view_sub = rospy.Subscriber("FlyView",FlyView,self.fly_view_callback)
        self.fly_view = FlyView()
        self.initialized = False

    def fly_view_callback(self,data):
        self.fly_view = data
        if not self.initialized:
            self.initialized = True

class KinematicsSubscriber:
    def __init__(self):
        self.robot_fly_kinematics_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.kinematics_callback)
        self.kinematics = RobotFlyKinematics()
        self.initialized = False

    def kinematics_callback(self,data):
        self.kinematics = data
        if not self.initialized:
            self.initialized = True
