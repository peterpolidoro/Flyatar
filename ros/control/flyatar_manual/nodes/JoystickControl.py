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
        self.sc_pub = rospy.Publisher("Stage/Commands",StageCommands)

        self.stage_commands = StageCommands()
        self.robot_velocity_max = rospy.get_param("robot_velocity_max",100) # mm/s
        self.moving_to_start = False
        self.start_x_position = 125     # mm in Stage coordinates
        self.start_y_position = 125     # mm in Stage coordinates
        self.goto_start_velocity = 50   # mm/s
        self.velocity_threshold = 0.01
        self.initialized = True

    def commands_callback(self,data):
        if self.initialized:
            if not self.moving_to_start:
                if data.goto_start:
                    self.moving_to_start = True
                    self.stage_commands.x_position = [self.start_x_position]
                    self.stage_commands.y_position = [self.start_y_position]
                    self.stage_commands.x_velocity = []
                    self.stage_commands.y_velocity = []
                    self.stage_commands.velocity_magnitude = [self.goto_start_velocity]
                else:
                    self.stage_commands.x_position = []
                    self.stage_commands.y_position = []
                    self.stage_commands.x_velocity = [data.y_velocity]
                    self.stage_commands.y_velocity = [-data.x_velocity]
                    self.stage_commands.velocity_magnitude = [self.robot_velocity_max]
            elif (self.velocity_threshold < abs(data.x_velocity)) or (self.velocity_threshold < abs(data.y_velocity)):
                self.moving_to_start = False

            self.sc_pub.publish(self.stage_commands)


if __name__ == '__main__':
    rospy.init_node('JoystickControl', anonymous=True)
    jc = JoystickControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
