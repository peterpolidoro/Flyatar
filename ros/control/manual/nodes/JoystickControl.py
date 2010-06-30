#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('manual')
import sys
import rospy
from stage.msg import StageCommands
from joystick_commands.msg import JoystickCommands

class PositionControl:

    def __init__(self):
        self.initialized = False

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.commands_callback)
        self.sc_pub = rospy.Publisher("StageCommands",StageCommands)

        self.stage_commands = StageCommands()
        self.stage_commands.position_control = False
        self.vel_scale_factor = 100     # mm/s
        self.initialized = True

    def commands_callback(self,data):
        if self.initialized:
            self.stage_commands.x_velocity = data.y_velocity*self.vel_scale_factor
            self.stage_commands.y_velocity = -data.x_velocity*self.vel_scale_factor

            self.sc_pub.publish(self.stage_commands)


if __name__ == '__main__':
    rospy.init_node('JoystickControl', anonymous=True)
    jc = JoystickControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
