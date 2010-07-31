#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_manual')
import sys
import rospy
from stage.msg import StageCommands
from joystick_commands.msg import JoystickCommands

class JoystickControl:

    def __init__(self):
        self.initialized = False

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.commands_callback)
        self.sc_pub = rospy.Publisher("StageCommands",StageCommands)

        self.stage_commands = StageCommands()
        self.stage_commands.position_control = False
        self.robot_velocity_max = rospy.get_param("robot_velocity_max",100) # mm/s
        self.initialized = True

    def commands_callback(self,data):
        if self.initialized:
            self.stage_commands.position_control = False
            self.stage_commands.velocity_control = True
            self.stage_commands.lookup_table_correct = False
            self.stage_commands.x_velocity = [data.y_velocity*self.robot_velocity_max]
            self.stage_commands.y_velocity = [-data.x_velocity*self.robot_velocity_max]

            self.sc_pub.publish(self.stage_commands)


if __name__ == '__main__':
    rospy.init_node('JoystickControl', anonymous=True)
    jc = JoystickControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
