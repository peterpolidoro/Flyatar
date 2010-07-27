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

    def _fill_response(self,return_state):
        x,y,theta,x_velocity,y_velocity,theta_velocity,all_motors_in_position,lookup_table_move_complete = return_state
        self.response.header.stamp = rospy.Time.now()
        self.response.x = x
        self.response.y = y
        self.response.theta = theta
        self.response.x_velocity = x_velocity
        self.response.y_velocity = y_velocity
        self.response.theta_velocity = theta_velocity
        self.response.all_motors_in_position = bool(all_motors_in_position)
        self.response.lookup_table_move_complete = bool(lookup_table_move_complete)

    def close(self):
        self.dev.close()
        print "Flyatar stage device closed."

    def get_stage_state(self,req):
        # return_state = self.dev.return_state()
        return_state = self.dev.get_state()
        self._fill_response(return_state)
        return self.response

    def set_stage_velocity(self,req):
        # x_velocity = req.x_velocity
        # y_velocity = req.y_velocity
        # if abs(x_velocity) < self.min_velocity:
        #     x_velocity = 0
        # if abs(y_velocity) < self.min_velocity:
        #     y_velocity = 0
        # return_state = self.dev.update_velocity(x_velocity,y_velocity)
        x_vel_list = req.x_velocity
        y_vel_list = req.y_velocity
        # if abs(x_velocity) < self.min_velocity:
        #     x_velocity = 0
        # if abs(y_velocity) < self.min_velocity:
        #     y_velocity = 0
        return_state = self.dev.update_velocity(x_vel_list,y_vel_list)
        self._fill_response(return_state)
        return self.response

    def set_stage_position(self,req):
        # x_position = req.x_position
        # y_position = req.y_position
        # x_velocity = req.x_velocity
        # y_velocity = req.y_velocity
        # return_state = self.dev.update_position(x_position,x_velocity,y_position,y_velocity)
        x_pos_list = req.x_position
        y_pos_list = req.y_position
        x_vel_list = req.x_velocity
        y_vel_list = req.y_velocity
        # point_count = min(len(x_pos_list),len(y_pos_list),len(x_vel_list),len(y_vel_list))
        # rospy.logwarn("point_count = %s" % (str(point_count)))
        return_state = self.dev.update_position(x_pos_list,x_vel_list,y_pos_list,y_vel_list)
        self._fill_response(return_state)
        return self.response

    # def stage_lookup_table_move(self,req):
    #     x_pos_list = [120,140,120,100,120,100,100,140,140]*3
    #     x_vel_list = [20]*len(x_pos_list)
    #     y_pos_list = [100,120,140,120,100,100,140,140,100]*3
    #     y_vel_list = [20]*len(y_pos_list)

    #     # x_pos_list = [120,140,120,100,120]
    #     # x_vel_list = [20,20,20,20,20]
    #     # y_pos_list = [100,120,140,120,100]
    #     # y_vel_list = [20,20,20,20,20]

    #     return_state = self.dev.lookup_table_move(x_pos_list,x_vel_list,y_pos_list,y_vel_list)
    #     self._fill_response(return_state)
    #     return self.response

if __name__ == '__main__':
    rospy.init_node('StageCommunicator', anonymous=True)
    sc = StageCommunicator()
    s_gss = rospy.Service('get_stage_state', Stage_State, sc.get_stage_state)
    s_ssv = rospy.Service('set_stage_velocity', Stage_State, sc.set_stage_velocity)
    s_ssp = rospy.Service('set_stage_position', Stage_State, sc.set_stage_position)
    # s_sltm = rospy.Service('stage_lookup_table_move', Stage_State, sc.stage_lookup_table_move)

    while not rospy.is_shutdown():
        rospy.spin()

    sc.close()
