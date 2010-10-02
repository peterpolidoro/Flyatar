#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics,InBounds
import math

class MonitorBounds:
    def __init__(self):
        self.in_bounds_radius = rospy.get_param('in_bounds_radius',100)
        self.robot_fly_kinematics_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.kinematics_callback)
        self.in_bounds_pub = rospy.Publisher("InBounds",InBounds)

        self.in_bounds = InBounds()

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


if __name__ == '__main__':
    rospy.init_node('MonitorBounds')
    while not rospy.is_shutdown():
        rospy.spin()
