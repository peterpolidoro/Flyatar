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
import USBDevice
import ctypes
import time

# Flyatar stage device parameters
_motor_num = 3
_entries_max = 5

# Input/Output Structures
class MotorState_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('Frequency', ctypes.c_uint16),
               ('Position', ctypes.c_uint16)]

# class USBPacketOut_t(ctypes.LittleEndianStructure):
#     _pack_ = 1
#     _fields_ =[('MotorUpdate', ctypes.c_uint8),
#                ('SetPoint', MotorState_t * _motor_num)]

class LookupTableRow_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('MotorSetPoint', MotorState_t * _motor_num)]

class USBPacketOut_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('MotorUpdate', ctypes.c_uint8),
               ('EntryCount', ctypes.c_uint8),
               ('EntryLocation', ctypes.c_uint8),
               ('Entry', LookupTableRow_t * _entries_max)]

# Firmware Code for Reference:
# typedef MotorStatus_t LookupTableRow_t[MOTOR_NUM];
# typedef struct
# {
#   uint8_t       CommandID;
#   uint8_t       MotorUpdate;
#   uint8_t       EntryCount;
#   uint8_t       EntryLocation;
#   LookupTableRow_t Setpoint[ENTRIES_MAX];
# } USBPacketOutWrapper_t;

class USBPacketIn_t(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ =[('MotorState', MotorState_t * _motor_num)]

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
        self.USB_CMD_LOOKUP_TABLE_MOVE = ctypes.c_uint8(4)

        self.USBPacketOut = USBPacketOut_t()
        self.USBPacketIn = USBPacketIn_t()
        # self.Motor = []
        # for MotorN in range(_motor_num):
        #     self.Motor.append({'Frequency'        : 0,
        #                        'FrequencyMax'     : FREQUENCY_MAX,
        #                        'Position'         : 0,
        #                        'PositionMin'      : POSITION_MIN,
        #                        'PositionMax'      : POSITION_MAX,
        #                        'PositionSetPoint' : 0,
        #                        'Direction'        : 0})

        # Parameters
        self.frequency_max = 30000
        self.position_min = 0
        self.position_max = 44000

        self.steps_per_mm = 5000/25.4   # 5000 steps per inch
                                        # 25.4 mm per inch
        self.steps_per_radian = 200     # Change to actual number!
        self.axis_x = 0
        self.axis_y = 1
        self.axis_theta = 2
        self.x_vel_mm = 0
        self.x_vel_steps = 0
        self.y_vel_mm = 0
        self.y_vel_steps = 0
        self.x_pos_mm = 0
        self.x_pos_steps = 0
        self.y_pos_mm = 0
        self.y_pos_steps = 0

    def update_velocity(self, x_velocity, y_velocity):
        self.x_vel_mm = x_velocity
        self.y_vel_mm = y_velocity
        self.x_vel_steps = self._mm_to_steps(self.x_vel_mm)
        self.y_vel_steps = self._mm_to_steps(self.y_vel_mm)

        if self.x_vel_steps < 0:
            self.x_pos_steps = self.position_min
            self.x_vel_steps = abs(self.x_vel_steps)
        else:
            self.x_pos_steps = self.position_max

        if self.y_vel_steps < 0:
            self.y_pos_steps = self.position_min
            self.y_vel_steps = abs(self.y_vel_steps)
        else:
            self.y_pos_steps = self.position_max

        if self.x_vel_steps > self.frequency_max:
            self.x_vel_steps = self.frequency_max

        if self.y_vel_steps > self.frequency_max:
            self.y_vel_steps = self.frequency_max

        self._set_frequency(self.axis_x,self.x_vel_steps)
        self._set_position(self.axis_x,self.x_pos_steps)
        self._set_frequency(self.axis_y,self.y_vel_steps)
        self._set_position(self.axis_y,self.y_pos_steps)
        self._set_motor_state()
        x,y,theta,x_velocity,y_velocity,theta_velocity = self.return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def update_position(self,x_position,y_position,x_velocity,y_velocity):
        self.x_pos_mm = x_position
        self.y_pos_mm = y_position
        self.x_pos_steps = self._mm_to_steps(self.x_pos_mm)
        self.y_pos_steps = self._mm_to_steps(self.y_pos_mm)
        self.x_vel_mm = x_velocity
        self.y_vel_mm = y_velocity
        self.x_vel_steps = self._mm_to_steps(self.x_vel_mm)
        self.y_vel_steps = self._mm_to_steps(self.y_vel_mm)

        if self.x_pos_steps < self.position_min:
            self.x_pos_steps = self.position_min
        elif self.position_max < self.x_pos_steps:
            self.x_pos_steps = self.position_max
        if self.y_pos_steps < self.position_min:
            self.y_pos_steps = self.position_min
        elif self.position_max < self.y_pos_steps:
            self.y_pos_steps = self.position_max

        if self.frequency_max < self.x_vel_steps:
            self.y_vel_steps = (self.y_vel_steps/self.x_velocity_steps)*self.frequency_max
            self.x_vel_steps = self.frequency_max
        if self.frequency_max < self.y_vel_steps:
            self.x_vel_steps = (self.x_vel_steps/self.y_velocity_steps)*self.frequency_max
            self.y_vel_steps = self.frequency_max

        self._set_frequency(self.axis_x,self.x_vel_steps)
        self._set_position(self.axis_x,self.x_pos_steps)
        self._set_frequency(self.axis_y,self.y_vel_steps)
        self._set_position(self.axis_y,self.y_pos_steps)
        self._set_motor_state()
        x,y,theta,x_velocity,y_velocity,theta_velocity = self.return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def lookup_table_move(self):
        self._lookup_table_fill()
        self._lookup_table_move()
        x,y,theta,x_velocity,y_velocity,theta_velocity = self.return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def get_state(self):
        self._get_motor_state()
        x,y,theta,x_velocity,y_velocity,theta_velocity = self.return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def return_state(self):
        x_velocity = self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_x].Frequency)
        x = self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_x].Position)
        y_velocity =  self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_y].Frequency)
        y =  self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_y].Position)
        theta_velocity = self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_theta].Frequency)
        theta =  self._steps_to_mm(self.USBPacketIn.MotorState[self.axis_theta].Position)
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def _mm_to_steps(self,quantity_mm):
        return quantity_mm*self.steps_per_mm

    def _steps_to_mm(self,quantity_steps):
        return quantity_steps/self.steps_per_mm

    # def _set_frequency(self,axis,freq):
    #     self.USBPacketOut.SetPoint[axis].Frequency = int(freq)

    # def _set_position(self,axis,pos):
    #     self.USBPacketOut.SetPoint[axis].Position = int(pos)

    def _set_frequency(self,axis,freq,entry_n=0):
        self.USBPacketOut.Entry[entry_n].MotorSetPoint[axis].Frequency = int(freq)

    def _set_position(self,axis,pos,entry_n=0):
        self.USBPacketOut.Entry[entry_n].MotorSetPoint[axis].Position = int(pos)

    def _get_motor_state(self):
        outdata = [self.USB_CMD_GET_STATE]
        intypes = [ctypes.c_uint8, USBPacketIn_t]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        self._check_cmd_id(self.USB_CMD_GET_STATE,cmd_id)
        self.USBPacketIn = val_list[1]

    def _set_motor_state(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        outdata = [self.USB_CMD_SET_STATE, self.USBPacketOut]
        intypes = [ctypes.c_uint8, USBPacketIn_t]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        self._check_cmd_id(self.USB_CMD_SET_STATE,cmd_id)
        self.USBPacketIn = val_list[1]

    def _lookup_table_fill(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        outdata = [self.USB_CMD_LOOKUP_TABLE_FILL, self.USBPacketOut]
        intypes = [ctypes.c_uint8, USBPacketIn_t]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        self._check_cmd_id(self.USB_CMD_LOOKUP_TABLE_FILL,cmd_id)
        self.USBPacketIn = val_list[1]

    def _lookup_table_move(self):
        self.USBPacketOut.MotorUpdate = ctypes.c_uint8(7)
        outdata = [self.USB_CMD_LOOKUP_TABLE_MOVE, self.USBPacketOut]
        intypes = [ctypes.c_uint8, USBPacketIn_t]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id = val_list[0]
        self._check_cmd_id(self.USB_CMD_LOOKUP_TABLE_MOVE,cmd_id)
        self.USBPacketIn = val_list[1]

    def _print_motor_state(self):
        print '*'*20
        print 'Frequency X = ', self.USBPacketIn.MotorState[self.axis_x].Frequency
        print 'Position X = ', self.USBPacketIn.MotorState[self.axis_x].Position
        print 'Frequency Y = ', self.USBPacketIn.MotorState[self.axis_y].Frequency
        print 'Position Y = ', self.USBPacketIn.MotorState[self.axis_y].Position
        print 'Frequency Theta = ', self.USBPacketIn.MotorState[self.axis_theta].Frequency
        print 'Position Theta = ', self.USBPacketIn.MotorState[self.axis_theta].Position
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
