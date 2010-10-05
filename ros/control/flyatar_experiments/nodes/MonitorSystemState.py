#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics,InBounds
import tf
import math
from geometry_msgs.msg import PointStamped

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

class FlyState:
    def __init__(self):
        self.robot_fly_kinematics_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.kinematics_callback)
        self.tf_listener = tf.TransformListener()

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0

        self.initialized = False

    def kinematics_callback(self,data):
        self.robot_stopped = data.robot_stopped
        self.fly_stopped = data.fly_stopped
        self.robot_position_x = data.robot_kinematics.position.x
        self.robot_position_y = data.robot_kinematics.position.y
        self.fly_position_x = data.fly_kinematics.position.x
        self.fly_position_y = data.fly_kinematics.position.y
        self.robot_velocity_x = data.robot_kinematics.velocity.x
        self.robot_velocity_y = data.robot_kinematics.velocity.y
        self.fly_velocity_x = data.fly_kinematics.velocity.x
        self.fly_velocity_y = data.fly_kinematics.velocity.y
        self.robot_fly_dist = math.sqrt((self.robot_position_x - self.fly_position_x)**2 + (self.robot_position_y - self.fly_position_y)**2)
        try:
            self.robot_origin_fly_frame = self.tf_listener.transformPoint("Fly",self.robot_origin)
            self.robot_in_front_of_fly = 0 < self.robot_origin_fly_frame.point.y
            if not self.initialized:
                self.initialized = True
        except (tf.LookupException, tf.ConnectivityException):
            pass
