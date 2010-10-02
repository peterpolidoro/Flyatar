#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics

class MonitorBounds:
    def __init__(self):
        self.robot_fly_kinematics_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.kinematics_callback)

    def kinematics_callback(self,data):
        pass

if __name__ == '__main__':
    rospy.init_node('MonitorBounds')
    while not rospy.is_shutdown():
        rospy.spin()
