#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import StageDevice
from stage.srv import *
import threading

class StageCommunicator():
    def __init__(self):
        rospy.loginfo("Opening Flyatar stage device...")
        self.dev = StageDevice.StageDevice()
        self.dev.print_values()
        self.response = Stage_StateResponse()
        self.reentrant_lock = threading.Lock()

    def _fill_response(self,return_state):
        x,y,theta,x_velocity,y_velocity,theta_velocity,all_motors_in_position,lookup_table_move_in_progress = return_state
        self.response.header.stamp = rospy.Time.now()
        self.response.x = x
        self.response.y = y
        self.response.theta = theta
        self.response.x_velocity = x_velocity
        self.response.y_velocity = y_velocity
        self.response.theta_velocity = theta_velocity
        self.response.all_motors_in_position = bool(all_motors_in_position)
        self.response.lookup_table_move_in_progress = lookup_table_move_in_progress

    def close(self):
        self.dev.close()
        rospy.loginfo("Flyatar stage device closed.")

    def get_stage_state(self,req):
        with self.reentrant_lock:
            return_state = self.dev.get_state()
            self._fill_response(return_state)
        return self.response

    def set_stage_velocity(self,req):
        x_vel_list = list(req.x_velocity)
        y_vel_list = req.y_velocity
        vel_mag_list = req.velocity_magnitude
        if (len(x_vel_list) == 0) or (len(y_vel_list) == 0):
            rospy.logerr("Error in set_stage_velocity call: x_vel_list or y_vel_list length == 0")
        else:
            with self.reentrant_lock:
                return_state = self.dev.update_velocity(x_vel_list,y_vel_list,vel_mag_list)
                self._fill_response(return_state)
            return self.response

    def set_stage_position(self,req):
        x_pos_list = req.x_position
        y_pos_list = req.y_position
        vel_mag_list = req.velocity_magnitude
        if (len(x_pos_list) == 0) or (len(y_pos_list) == 0) or (len(vel_mag_list) == 0):
            rospy.logerr("Error in set_stage_position call: x_pos_list or y_pos_list or vel_mag_list length == 0")
        else:
            with self.reentrant_lock:
                return_state = self.dev.update_position(x_pos_list,y_pos_list,vel_mag_list)
                self._fill_response(return_state)
            return self.response

if __name__ == '__main__':
    rospy.init_node('StageCommunicator', anonymous=True)
    sc = StageCommunicator()
    s_gss = rospy.Service('get_stage_state', Stage_State, sc.get_stage_state)
    s_ssv = rospy.Service('set_stage_velocity', Stage_State, sc.set_stage_velocity)
    s_ssp = rospy.Service('set_stage_position', Stage_State, sc.set_stage_position)

    while not rospy.is_shutdown():
        rospy.spin()

    sc.close()
