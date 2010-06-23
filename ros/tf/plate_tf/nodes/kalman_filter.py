#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy

import cv, numpy
from geometry_msgs.msg import PoseStamped

class KalmanFilter:
    def __init__(self):
        self.kal = cv.CreateKalman(4,2,0)
        self.t_previous = None

    def update(self,z,t):
        self.t_current = t
        self.update_dt()

    def update_dt(self):
        if self.t_previous is not None:
            self.dt = self.t_current - self.t_previous
            rospy.logwarn("dt = %s", str(self.dt))
        self.t_previous = self.t_current
