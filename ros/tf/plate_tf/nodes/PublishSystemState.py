#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics,InBounds,FlyView
from geometry_msgs.msg import PointStamped
from pythonmodules import CircleFunctions
import tf
import math

class PublishSystemState:
    def __init__(self):
        self.in_bounds_radius = rospy.get_param('in_bounds_radius')
        self.robot_fly_kinematics_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.kinematics_callback)
        self.tf_listener = tf.TransformListener()

        self.in_bounds_pub = rospy.Publisher("InBounds",InBounds)
        self.fly_view_pub = rospy.Publisher("FlyView",FlyView)

        self.in_bounds = InBounds()
        self.fly_view = FlyView()

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0

    def kinematics_callback(self,data):
        robot_x = data.robot_kinematics.position.x
        robot_y = data.robot_kinematics.position.y
        fly_x = data.fly_kinematics.position.x
        fly_y = data.fly_kinematics.position.y
        robot_dist = math.sqrt(robot_x**2 + robot_y**2)
        fly_dist = math.sqrt(fly_x**2 + fly_y**2)
        self.in_bounds.bounds_radius = self.in_bounds_radius
        self.in_bounds.robot_in_bounds = robot_dist < self.in_bounds_radius
        self.in_bounds.fly_in_bounds = fly_dist < self.in_bounds_radius
        self.in_bounds_pub.publish(self.in_bounds)

        try:
            self.robot_origin_fly_frame = self.tf_listener.transformPoint("Fly",self.robot_origin)
            self.fly_view.robot_position_x = rx = self.robot_origin_fly_frame.point.x
            self.fly_view.robot_position_y = ry = self.robot_origin_fly_frame.point.y
            self.fly_view.robot_angle = CircleFunctions.mod_angle(math.atan2(ry,rx))
            self.fly_view.robot_distance = math.sqrt(rx**2 + ry**2)
            self.fly_view.robot_in_front_of_fly = 0 < self.robot_origin_fly_frame.point.x
            self.fly_view_pub.publish(self.fly_view)
        except (tf.LookupException, tf.ConnectivityException):
            pass

if __name__ == '__main__':
    rospy.init_node('PublishSystemState')
    pss = PublishSystemState()
    while not rospy.is_shutdown():
        rospy.spin()
