#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import StageDevice
from stage.srv import *

class StageCommunicator():
    def __init__(self):
        print "Opening Flyatar stage device..."
        self.dev = StageDevice.StageDevice()
        self.dev.print_values()
        self.response = Stage_StateResponse()
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
        print "Flyatar stage device closed."

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

    def set_stage_position(self,req):
        x_position = req.x_position
        y_position = req.y_position
        x_velocity = req.x_velocity
        y_velocity = req.y_velocity
        return_state = self.dev.update_position(x_position,y_position,x_velocity,y_velocity)
        self._fill_response(return_state)
        return self.response

    def stage_lookup_table_move(self,req):
        # vel_steps = self._mm_to_steps(20)
        # pos_steps_100 = self._mm_to_steps(100)
        # pos_steps_120 = self._mm_to_steps(120)
        # pos_steps_140 = self._mm_to_steps(140)

        # self._set_frequency(self.axis_x,vel_steps,0)
        # self._set_position(self.axis_x,pos_steps_120,0)
        # self._set_frequency(self.axis_y,vel_steps,0)
        # self._set_position(self.axis_y,pos_steps_100,0)

        # self._set_frequency(self.axis_x,vel_steps,1)
        # self._set_position(self.axis_x,pos_steps_140,1)
        # self._set_frequency(self.axis_y,vel_steps,1)
        # self._set_position(self.axis_y,pos_steps_120,1)

        # self._set_frequency(self.axis_x,vel_steps,2)
        # self._set_position(self.axis_x,pos_steps_120,2)
        # self._set_frequency(self.axis_y,vel_steps,2)
        # self._set_position(self.axis_y,pos_steps_140,2)

        # self._set_frequency(self.axis_x,vel_steps,3)
        # self._set_position(self.axis_x,pos_steps_100,3)
        # self._set_frequency(self.axis_y,vel_steps,3)
        # self._set_position(self.axis_y,pos_steps_120,3)

        # self._set_frequency(self.axis_x,vel_steps,4)
        # self._set_position(self.axis_x,pos_steps_120,4)
        # self._set_frequency(self.axis_y,vel_steps,4)
        # self._set_position(self.axis_y,pos_steps_100,4)

        x_pos_list = [120,140,120,100,120,100,100,140,140]*12
        x_vel_list = [20]*len(x_pos_list)
        y_pos_list = [100,120,140,120,100,100,140,140,100]*12
        y_vel_list = [20]*len(y_pos_list)

        # x_pos_list = [120,140,120,100,120]
        # x_vel_list = [20,20,20,20,20]
        # y_pos_list = [100,120,140,120,100]
        # y_vel_list = [20,20,20,20,20]

        return_state = self.dev.lookup_table_move(x_pos_list,x_vel_list,y_pos_list,y_vel_list)
        self._fill_response(return_state)
        return self.response

if __name__ == '__main__':
    rospy.init_node('StageCommunicator', anonymous=True)
    sc = StageCommunicator()
    s_gss = rospy.Service('get_stage_state', Stage_State, sc.get_stage_state)
    s_ssv = rospy.Service('set_stage_velocity', Stage_State, sc.set_stage_velocity)
    s_ssp = rospy.Service('set_stage_position', Stage_State, sc.set_stage_position)
    s_sltm = rospy.Service('stage_lookup_table_move', Stage_State, sc.stage_lookup_table_move)

    while not rospy.is_shutdown():
        rospy.spin()

    sc.close()
