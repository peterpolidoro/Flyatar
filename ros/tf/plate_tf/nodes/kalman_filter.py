#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy

import cv
from geometry_msgs.msg import PoseStamped

class KalmanFilter:
    def __init__(self):
        self.kal = cv.CreateKalman(4,2,0)
        cv.SetIdentity(self.kal.transition_matrix)
        cv.SetIdentity(self.kal.measurement_matrix)
        self.measurement = cv.CreateMat(2,1,cv.GetElemType(self.kal.state_pre))
        self.t_previous = None

    def update(self,z,t):
        self.t_current = t
        x = y = vx = vy = None
        if self.update_dt():
            cv.KalmanPredict(self.kal)
            self.measurement[0,0] = z[0]
            self.measurement[1,0] = z[1]
            state_post = cv.KalmanCorrect(self.kal,self.measurement)
            x = state_post[0,0]
            y = state_post[1,0]
            vx = state_post[2,0]
            vy = state_post[3,0]
        return (x,y,vx,vy)

    def update_dt(self):
        status = False
        if self.t_previous is not None:
            self.dt = self.t_current - self.t_previous
            self.kal.transition_matrix[0,2] = self.dt
            self.kal.transition_matrix[1,3] = self.dt
            status = True
        self.t_previous = self.t_current
        return status
