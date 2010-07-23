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
import USBDevice
import ctypes
import time
import math

# Flyatar stage device parameters
_motor_num = 3
_entries_max = 5
_lookup_table_size = 100
_lookup_table_update_freq = 62.5
rospy.set_param('lookup_table_update_freq',_lookup_table_update_freq)

# Input/Output Structures
class MotorState_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('Frequency', ctypes.c_uint16),
               ('Position', ctypes.c_uint16)]

class LookupTableRow_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('Motor', MotorState_t * _motor_num)]

class USBPacketOut_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('MotorUpdate', ctypes.c_uint8),
               ('EntryCount', ctypes.c_uint8),
               ('EntryLocation', ctypes.c_uint8),
               ('Entry', LookupTableRow_t * _entries_max)]

class USBPacketIn_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('AllMotorsInPosition', ctypes.c_uint8),
               ('LookupTableMoveComplete', ctypes.c_uint8),
               ('State', LookupTableRow_t)]

class StageDevice(USBDevice.USB_Device):
    def __init__(self, serial_number=None):

        # USB device parameters
        self.vendor_id = 0x0004
        self.product_id = 0x0002
        self.bulkout_ep_address = 0x01
        self.bulkin_ep_address = 0x82
        self.buffer_out_size = 64
        self.buffer_in_size = 64
        self.serial_number = serial_number

        USBDevice.USB_Device.__init__(self,
                                      self.vendor_id,
                                      self.product_id,
                                      self.bulkout_ep_address,
                                      self.bulkin_ep_address,
                                      self.buffer_out_size,
                                      self.buffer_in_size,
                                      self.serial_number)

        # USB Command IDs
        self.USB_CMD_GET_STATE = ctypes.c_uint8(1)
        self.USB_CMD_SET_STATE = ctypes.c_uint8(2)
        self.USB_CMD_LOOKUP_TABLE_FILL = ctypes.c_uint8(3)
        self.USB_CMD_LOOKUP_TABLE_POS_MOVE = ctypes.c_uint8(4)
        self.USB_CMD_LOOKUP_TABLE_VEL_MOVE = ctypes.c_uint8(5)

        self.USBPacketOut = USBPacketOut_t()
        self.USBPacketIn = USBPacketIn_t()

        # Parameters
        self.frequency_max = 30000
        self.position_min = 0
        self.position_max = 44000
        self.min_vel_mm = 1

        self.steps_per_mm = 5000/25.4   # 5000 steps per inch
                                        # 25.4 mm per inch
        self.steps_per_radian = 200     # Change to actual number!
        self.axis_x = 0
        self.axis_y = 1
        self.axis_theta = 2

    def update_velocity(self,x_vel_list,y_vel_list):
        # x_vel_mm = x_vel_list[0]
        # y_vel_mm = y_vel_list[0]
        # self._convert_and_set_setpoint(None,x_vel_mm,None,y_vel_mm,0)
        # self._set_motor_state()
        # x,y,theta,x_velocity,y_velocity,theta_velocity = self._return_state()
        # return x,y,theta,x_velocity,y_velocity,theta_velocity
        x,y,theta,x_velocity,y_velocity,theta_velocity = self._send_cmds_receive_status(None,x_vel_list,None,y_vel_list)
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def update_position(self,x_pos_list,x_vel_list,y_pos_list,y_vel_list):
        x,y,theta,x_velocity,y_velocity,theta_velocity = self._send_cmds_receive_status(x_pos_list,x_vel_list,y_pos_list,y_vel_list)
        return x,y,theta,x_velocity,y_velocity,theta_velocity

        # point_count = min(len(x_pos_list),len(x_vel_list),len(y_pos_list),len(y_vel_list))
        # if _lookup_table_size < point_count:
        #     point_count = _lookup_table_size

        # if point_count == 1:
        #     x_pos_mm = x_pos_list[0]
        #     x_vel_mm = x_vel_list[0]
        #     y_pos_mm = y_pos_list[0]
        #     y_vel_mm = y_vel_list[0]
        #     self._convert_and_set_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,0)
        #     self._set_motor_state()
        # else:
        #     packet_count = int(math.ceil(point_count/_entries_max))
        #     point_n = 0
        #     for packet_n in range(packet_count):
        #         packet_point_n = 0
        #         while (packet_point_n < _entries_max) and (point_n < point_count):
        #             x_pos_mm = x_pos_list[point_n]
        #             x_vel_mm = x_vel_list[point_n]
        #             y_pos_mm = y_pos_list[point_n]
        #             y_vel_mm = y_vel_list[point_n]

        #             self._convert_and_set_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,packet_point_n)

        #             packet_point_n += 1
        #             point_n += 1

        #         self.USBPacketOut.EntryCount = packet_point_n
        #         self.USBPacketOut.EntryLocation = point_n - packet_point_n
        #         # rospy.logwarn("packet_n = %s, packet_point_n = %s, point_n = %s" % (str(packet_n),str(packet_point_n),str(point_n)))
        #         self._lookup_table_fill()

        #     self._lookup_table_move()

        # x,y,theta,x_velocity,y_velocity,theta_velocity = self._return_state()
        # return x,y,theta,x_velocity,y_velocity,theta_velocity

    def _send_cmds_receive_status(self,x_pos_list,x_vel_list,y_pos_list,y_vel_list):
        point_count = min(len(x_vel_list),len(y_vel_list))
        if _lookup_table_size < point_count:
            point_count = _lookup_table_size

        if (x_pos_list is None) or (y_pos_list is None):
            x_pos_list = [None]*point_count
            y_pos_list = [None]*point_count
            vel_move = True
            if 1 < point_count:
                rospy.logwarn("x_vel_list = \n%s" % (str(x_vel_list)))
                rospy.logwarn("y_vel_list = \n%s" % (str(y_vel_list)))

        if point_count == 1:
            x_pos_mm = x_pos_list[0]
            x_vel_mm = x_vel_list[0]
            y_pos_mm = y_pos_list[0]
            y_vel_mm = y_vel_list[0]
            self._convert_and_set_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,0)
            self._set_motor_state()
        else:
            packet_count = int(math.ceil(point_count/_entries_max))
            point_n = 0
            for packet_n in range(packet_count):
                packet_point_n = 0
                while (packet_point_n < _entries_max) and (point_n < point_count):
                    x_pos_mm = x_pos_list[point_n]
                    x_vel_mm = x_vel_list[point_n]
                    y_pos_mm = y_pos_list[point_n]
                    y_vel_mm = y_vel_list[point_n]

                    self._convert_and_set_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,packet_point_n)

                    packet_point_n += 1
                    point_n += 1

                self.USBPacketOut.EntryCount = packet_point_n
                self.USBPacketOut.EntryLocation = point_n - packet_point_n
                # rospy.logwarn("packet_n = %s, packet_point_n = %s, point_n = %s" % (str(packet_n),str(packet_point_n),str(point_n)))
                self._lookup_table_fill()

            if vel_move:
                self._lookup_table_vel_move()
            else:
                self._lookup_table_pos_move()

        x,y,theta,x_velocity,y_velocity,theta_velocity = self._return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity


    # def lookup_table_move(self,x_pos_list,x_vel_list,y_pos_list,y_vel_list):
    #     point_count = min(len(x_pos_list),len(x_vel_list),len(y_pos_list),len(y_vel_list))
    #     if _lookup_table_size < point_count:
    #         point_count = _lookup_table_size

    #     packet_count = int(math.ceil(point_count/_entries_max))
    #     point_n = 0
    #     for packet_n in range(packet_count):
    #         packet_point_n = 0
    #         while (packet_point_n < _entries_max) and (point_n < point_count):
    #             x_pos_mm = x_pos_list[point_n]
    #             x_vel_mm = x_vel_list[point_n]
    #             y_pos_mm = y_pos_list[point_n]
    #             y_vel_mm = y_vel_list[point_n]

    #             self._convert_and_set_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,packet_point_n)

    #             packet_point_n += 1
    #             point_n += 1

    #         self.USBPacketOut.EntryCount = packet_point_n
    #         self.USBPacketOut.EntryLocation = point_n - packet_point_n
    #         # rospy.logwarn("packet_n = %s, packet_point_n = %s, point_n = %s" % (str(packet_n),str(packet_point_n),str(point_n)))
    #         self._lookup_table_fill()

    #     self._lookup_table_move()
    #     x,y,theta,x_velocity,y_velocity,theta_velocity = self._return_state()
    #     return x,y,theta,x_velocity,y_velocity,theta_velocity

    def get_state(self):
        self._get_motor_state()
        x,y,theta,x_velocity,y_velocity,theta_velocity = self._return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def _return_state(self):
        x_velocity = self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_x].Frequency)
        x = self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_x].Position)
        y_velocity =  self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_y].Frequency)
        y =  self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_y].Position)
        theta_velocity = self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_theta].Frequency)
        theta =  self._steps_to_mm(self.USBPacketIn.State.Motor[self.axis_theta].Position)
        all_motors_in_position = self.USBPacketIn.AllMotorsInPosition
        # if all_motors_in_position:
        #     rospy.logwarn("All motors in position.")
        lookup_table_move_complete = self.USBPacketIn.LookupTableMoveComplete
        # if lookup_table_move_complete:
        #     rospy.logwarn("Lookup table move complete.")
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def _convert_and_set_setpoint(self,x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm,entry_n=0):
        x_pos_steps,x_vel_steps,y_pos_steps,y_vel_steps = self._check_and_convert_setpoint(x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm)

        self._set_position(self.axis_x,x_pos_steps,entry_n)
        self._set_frequency(self.axis_x,x_vel_steps,entry_n)
        self._set_position(self.axis_y,y_pos_steps,entry_n)
        self._set_frequency(self.axis_y,y_vel_steps,entry_n)

    def _check_and_convert_setpoint(self,x_pos_mm,x_vel_mm,y_pos_mm,y_vel_mm):
        x_vel_steps = self._mm_to_steps(x_vel_mm)
        y_vel_steps = self._mm_to_steps(y_vel_mm)

        # Set position values
        if (x_pos_mm is None) or (y_pos_mm is None):
            if x_vel_steps < 0:
                x_pos_steps = self.position_min
            else:
                x_pos_steps = self.position_max

            if y_vel_steps < 0:
                y_pos_steps = self.position_min
            else:
                y_pos_steps = self.position_max
        else:
            x_pos_steps = self._mm_to_steps(x_pos_mm)
            y_pos_steps = self._mm_to_steps(y_pos_mm)

            if x_pos_steps < self.position_min:
                x_pos_steps = self.position_min
            elif self.position_max < x_pos_steps:
                x_pos_steps = self.position_max

            if y_pos_steps < self.position_min:
                y_pos_steps = self.position_min
            elif self.position_max < y_pos_steps:
                y_pos_steps = self.position_max

        # Set velocity values
        x_vel_steps = abs(x_vel_steps)
        y_vel_steps = abs(y_vel_steps)

        if self.frequency_max < x_vel_steps:
            y_vel_steps = (y_vel_steps/x_vel_steps)*self.frequency_max
            x_vel_steps = self.frequency_max

        if self.frequency_max < y_vel_steps:
            x_vel_steps = (x_vel_steps/y_vel_steps)*self.frequency_max
            y_vel_steps = self.frequency_max

        return x_pos_steps,x_vel_steps,y_pos_steps,y_vel_steps

    def _mm_to_steps(self,quantity_mm):
        return quantity_mm*self.steps_per_mm

    def _steps_to_mm(self,quantity_steps):
        return quantity_steps/self.steps_per_mm

    def _set_frequency(self,axis,freq,entry_n=0):
        self.USBPacketOut.Entry[entry_n].Motor[axis].Frequency = int(freq)

    def _set_position(self,axis,pos,entry_n=0):
        self.USBPacketOut.Entry[entry_n].Motor[axis].Position = int(pos)

    def _send_usb_cmd(self,cmd,data_in_out_packet):
        if data_in_out_packet:
            outdata = [cmd, self.USBPacketOut]
        else:
            outdata = [cmd]
        intypes = [ctypes.c_uint8, USBPacketIn_t]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        self._check_cmd_id(cmd,cmd_id)
        self.USBPacketIn = val_list[1]

    def _get_motor_state(self):
        self._send_usb_cmd(self.USB_CMD_GET_STATE,False)

    def _set_motor_state(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_SET_STATE,True)

    def _lookup_table_fill(self):
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_FILL,True)

    def _lookup_table_pos_move(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_POS_MOVE,True)

    def _lookup_table_vel_move(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        self._send_usb_cmd(self.USB_CMD_LOOKUP_TABLE_VEL_MOVE,True)

    def _print_motor_state(self):
        print '*'*20
        print 'Frequency X = ', self.USBPacketIn.State.Motor[self.axis_x].Frequency
        print 'Position X = ', self.USBPacketIn.State.Motor[self.axis_x].Position
        print 'Frequency Y = ', self.USBPacketIn.State.Motor[self.axis_y].Frequency
        print 'Position Y = ', self.USBPacketIn.State.Motor[self.axis_y].Position
        print 'Frequency Theta = ', self.USBPacketIn.State.Motor[self.axis_theta].Frequency
        print 'Position Theta = ', self.USBPacketIn.State.Motor[self.axis_theta].Position
        print '*'*20

    def _check_cmd_id(self,expected_id,received_id):
        """
        Compares expected and received command ids.

        Arguments:
            expected_id = expected command id
            received_is = received command id

        Return: None
        """
        if not expected_id.value == received_id.value:
            msg = "received incorrect command ID %d expected %d"%(received_id.value,expected_id.value)
            raise IOError, msg
        return

#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    print "Opening Flyatar stage device ..."
    dev = StageDevice()
    dev.print_values()
    dev.close()
    print "Flyatar stage device closed."
