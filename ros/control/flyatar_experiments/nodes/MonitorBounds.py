#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
from plate_tf.msg import RobotFlyKinematics

class FindConditions:
    def __init__(self):
        self.contour_info_sub = rospy.Subscriber("RobotFlyKinematics",RobotFlyKinematics,self.contour_callback)


if __name__ == '__main__':
    rospy.init_node('MonitorBounds')
