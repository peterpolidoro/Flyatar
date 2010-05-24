#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_tf')
import sys
import rospy
import math
import tf
from stage.msg import Velocity,Commands
from geometry_msgs.msg import PoseStamped

class CommandListener:

  def __init__(self):
    self.tf_listener = tf.TransformListener()

    self.joy_sub = rospy.Subscriber("magnet/commands", Commands, self.commands_callback)
    self.vel_pub = rospy.Publisher("stage/command_velocity",Velocity)

    self.command_coordinates = rospy.get_param("Magnet_Command_Coordinates","Stage").lower()

    self.vel_values = Velocity()
    self.vel_scale_factor_lin = 100     # mm/s
    self.vel_scale_factor_ang = 5       # radians/s

  def commands_callback(self,data):
    r_velocity = data.r_velocity*self.vel_scale_factor_lin
    ang_velocity = data.ang_velocity*self.vel_scale_factor_ang
    x_velocity = data.x_velocity*self.vel_scale_factor_lin
    y_velocity = data.y_velocity*self.vel_scale_factor_lin

    if self.command_coordinates in 'stage':
      self.vel_values.x_velocity = x_velocity
      self.vel_values.y_velocity = y_velocity
      self.vel_pub.publish(self.vel_values)
    elif self.command_coordinates in 'plate':
      try:
        # (trans,rot) = self.tf_listener.lookupTransform('/Magnet', '/Plate', rospy.Time(0))
        (trans,rot) = self.tf_listener.lookupTransform('/Plate', '/Magnet', rospy.Time(0))
        x = trans[0]
        y = trans[1]
        theta = math.atan2(y,x)
        r = math.sqrt(x**2 + y**2)
        rospy.logwarn("r = %s",str(r))
        rospy.logwarn("theta = %s",str(theta))
        x_vel_plate = r_velocity*math.cos(theta) - r*ang_velocity*math.sin(theta)
        y_vel_plate = r_velocity*math.sin(theta) + r*ang_velocity*math.cos(theta)
        if self.vel_scale_factor_lin < abs(x_vel_plate):
          x_vel_plate = math.copysign(self.vel_scale_factor_lin,x_vel_plate)
        if self.vel_scale_factor_lin < abs(y_vel_plate):
          y_vel_plate = math.copysign(self.vel_scale_factor_lin,y_vel_plate)
        rospy.logwarn("x = %s",str(x))
        rospy.logwarn("y = %s",str(y))
        rospy.logwarn("r = %s",str(r))
        rospy.logwarn("theta = %s",str(theta))
        rospy.logwarn("r_velocity = %s",str(r_velocity))
        rospy.logwarn("ang_velocity = %s",str(ang_velocity))
        rospy.logwarn("x_vel_plate = %s",str(x_vel_plate))
        rospy.logwarn("y_vel_plate = %s",str(y_vel_plate))
        self.vel_values.x_velocity = y_vel_plate
        self.vel_values.y_velocity = -x_vel_plate
        self.vel_pub.publish(self.vel_values)
      except (tf.LookupException, tf.ConnectivityException):
        pass

if __name__ == '__main__':
  rospy.init_node('MagnetCommandListener', anonymous=True)
  cl = CommandListener()

  while not rospy.is_shutdown():
    rospy.spin()
