#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import StageDevice
from stage.srv import *

class StageCommunicator():
    def __init__(self):
        print "Opening XYFly stage device..."
        self.dev = StageDevice.StageDevice()
        self.dev.print_values()
        self.response = Velocity_StateResponse()
        self.min_velocity = 1

    def _fill_response(self,return_state):
        x,y,theta,x_velocity,y_velocity,theta_velocity = return_state
        self.response.header.stamp = rospy.Time.now()
        self.response.x = x
        self.response.y = y
        self.response.theta = theta
        self.response.x_velocity = x_velocity
        self.response.y_velocity = y_velocity
        self.response.theta_velocity = theta_velocity

    def close(self):
        self.dev.close()
        print "XYFly stage device closed."

    def get_stage_state(self,req):
        return_state = self.dev.return_state()
        self._fill_response(return_state)
        return self.response

    def set_stage_velocity(self,req):
        x_velocity = req.x_velocity
        y_velocity = req.y_velocity
        if abs(x_velocity) < self.min_velocity:
            x_velocity = 0
        if abs(y_velocity) < self.min_velocity:
            y_velocity = 0
        return_state = self.dev.update_velocity(x_velocity,y_velocity)
        self._fill_response(return_state)
        return self.response

if __name__ == '__main__':
    rospy.init_node('StageCommunicator', anonymous=True)
    sc = StageCommunicator()
    s_gss = rospy.Service('get_stage_state', Velocity_State, sc.get_stage_state)
    s_ssv = rospy.Service('set_stage_velocity', Velocity_State, sc.set_stage_velocity)

    while not rospy.is_shutdown():
        rospy.spin()

    sc.close()
