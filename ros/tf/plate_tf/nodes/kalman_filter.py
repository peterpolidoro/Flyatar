#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('plate_tf')
import rospy
from pythonmodules import CircleFunctions

import cv
from geometry_msgs.msg import PoseStamped

class KalmanFilter:
    def __init__(self):
        # self.kal = cv.CreateKalman(4,2,0)
        # self.kal = cv.CreateKalman(4,4,0)
        self.kal = cv.CreateKalman(6,6,0)
        cv.SetIdentity(self.kal.transition_matrix)
        cv.SetIdentity(self.kal.measurement_matrix)
        cv.SetIdentity(self.kal.process_noise_cov, 10)
        cv.SetIdentity(self.kal.measurement_noise_cov, 0.001)
        # self.kal.measurement_noise_cov[2,2] = 40
        # self.kal.measurement_noise_cov[3,3] = 40
        self.kal.measurement_noise_cov[2,2] = 100
        self.kal.measurement_noise_cov[3,3] = 40
        self.kal.measurement_noise_cov[4,4] = 40
        self.kal.measurement_noise_cov[5,5] = 10
        # self.measurement = cv.CreateMat(2,1,cv.GetElemType(self.kal.state_pre))
        # self.measurement = cv.CreateMat(4,1,cv.GetElemType(self.kal.state_pre))
        self.measurement = cv.CreateMat(6,1,cv.GetElemType(self.kal.state_pre))
        self.t_previous = None
        self.x_previous = None
        self.y_previous = None
        self.a_previous = None

    def update(self,z,t):
        self.t_current = t
        # x = y = vx = vy = None
        x = y = a = vx = vy = va = None
        if self.update_dt():
            cv.KalmanPredict(self.kal)
            # Q = self.kal.process_noise_cov[0,0]
            # R = self.kal.measurement_noise_cov[0,0]
            # rospy.logwarn("Q = %s, R = %s" % (str(Q),str(R)))
            self.measurement[0,0] = z[0]
            self.measurement[1,0] = z[1]
            self.measurement[2,0] = z[2]

            if self.x_previous is not None:
                # self.measurement[2,0] = (z[0] - self.x_previous)/self.dt
                # self.measurement[3,0] = (z[1] - self.y_previous)/self.dt
                self.measurement[3,0] = (z[0] - self.x_previous)/self.dt
                self.measurement[4,0] = (z[1] - self.y_previous)/self.dt
                self.measurement[5,0] = CircleFunctions.circle_dist(self.a_previous,z[2])/self.dt
                # rospy.logwarn("x_velocity = %s, y_velocity = %s" % (str(self.measurement[2,0]),str(self.measurement[3,0])))

            # rospy.logwarn("measurement[0,0] = %s" % str(self.measurement[0,0]))
            # rospy.logwarn("measurement[1,0] = %s" % str(self.measurement[1,0]))

            state_post = cv.KalmanCorrect(self.kal,self.measurement)
            x = state_post[0,0]
            y = state_post[1,0]
            a = state_post[2,0]
            # vx = state_post[2,0]
            # vy = state_post[3,0]
            vx = state_post[3,0]
            vy = state_post[4,0]
            va = state_post[5,0]

            self.x_previous = x
            self.y_previous = y
            self.a_previous = a

        return (x,y,a,vx,vy,va)
        # return (x,y,vx,vy)

    def update_dt(self):
        status = False
        if self.t_previous is not None:
            self.dt = self.t_current - self.t_previous
            # self.kal.transition_matrix[0,2] = self.dt
            # self.kal.transition_matrix[1,3] = self.dt
            self.kal.transition_matrix[0,3] = self.dt
            self.kal.transition_matrix[1,4] = self.dt
            self.kal.transition_matrix[2,5] = self.dt
            status = True
        self.t_previous = self.t_current
        return status
