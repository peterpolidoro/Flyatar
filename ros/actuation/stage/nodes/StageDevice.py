#!/usr/bin/env python
#
# StageDevice.py
#
# Control interface for at90usb based flyatar stage board.
# Provides python module and a command line utility.
#
# Note, need to set permissions correctly to get device to respond to nonroot
# users. This required adding and rules file to udev/rules.d and adding a
# group.
#
#    who when        what
#    --- ----        ----
#    pjp 08/19/09    version 1.0
# ---------------------------------------------------------------------------
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import MotorController
from stage.srv import *
import math

class StageDevice(MotorController.MotorControllerDevice):
    def __init__(self, serial_number=None):

        # Parameters in mm
        self.frequency_max_mm = 254
        self.position_min_mm = 0
        self.position_max_mm = 223.52

        self.steps_per_rev = 10000      # IM483 microstep setting
        self.in_per_rev = 2             # Timing belt pulley circumference
        self.mm_per_in = 25.4
        self.steps_per_mm = (self.steps_per_rev/self.in_per_rev)/self.mm_per_in
        self.steps_per_radian = self.steps_per_rev/math.pi

        rospy.set_param('lookup_table_update_freq',MotorController._lookup_table_update_freq)
        self.response = Stage_StateResponse()

        MotorController.MotorControllerDevice.__init__(self,
                                                       self._mm_to_steps(self.frequency_max_mm)[0],
                                                       self._mm_to_steps(self.position_min_mm)[0],
                                                       self._mm_to_steps(self.position_max_mm)[0])

    def get_stage_state(self):
        return_state = self.get_state()
        return self._fill_response(return_state)

    def update_stage_velocity(self,x_vel_mm,y_vel_mm,vel_mag_mm):
        motor0_vel = self._mm_to_steps(x_vel_mm)
        motor1_vel = self._mm_to_steps(y_vel_mm)
        vel_mag_steps = self._mm_to_steps(vel_mag_mm)
        return_state = self.update_velocity(motor0_vel,motor1_vel,vel_mag_steps)
        return self._fill_response(return_state)

    def update_stage_position(self,x_pos_mm,y_pos_mm,vel_mag_mm):
        motor0_pos = self._mm_to_steps(x_pos_mm)
        motor1_pos = self._mm_to_steps(y_pos_mm)
        vel_mag_steps = self._mm_to_steps(vel_mag_mm)
        return_state = self.update_position(motor0_pos,motor1_pos,vel_mag_steps)
        return self._fill_response(return_state)

    def _mm_to_steps(self,quantity_mm):
        try:
            list_mm = list(quantity_mm)
        except:
            list_mm = [quantity_mm]
        return [int(round(quantity_mm*self.steps_per_mm)) for quantity_mm in list_mm]

    def _steps_to_mm(self,quantity_steps):
        try:
            list_steps = list(quantity_steps)
        except:
            list_steps = [quantity_steps]
        return [(quantity_steps/self.steps_per_mm) for quantity_steps in list_steps]

    def _fill_response(self,return_state):
        rospy.logwarn("return_state = %s" % (str(return_state)))
        pos_vel_values_step = return_state[0:6]
        pos_vel_values_mm = self._steps_to_mm(pos_vel_values_step)
        x,x_velocity,y,y_velocity,theta,theta_velocity = pos_vel_values_mm
        all_motors_in_position = bool(return_state[6])
        lookup_table_move_in_progress = bool(return_state[7])
        motors_homed = bool(return_state[8])
        self.response.header.stamp = rospy.Time.now()
        self.response.x = x
        self.response.y = y
        self.response.theta = theta
        self.response.x_velocity = x_velocity
        self.response.y_velocity = y_velocity
        self.response.theta_velocity = theta_velocity
        self.response.all_motors_in_position = all_motors_in_position
        self.response.lookup_table_move_in_progress = lookup_table_move_in_progress
        self.response.motors_homed = motors_homed
        rospy.logwarn("response = %s" % (str(self.response)))
        return self.response

#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.loginfo("Opening Flyatar stage device ...")
    dev = StageDevice()
    dev.print_values()
    dev.close()
    rospy.loginfo("Flyatar stage device closed.")
