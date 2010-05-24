#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_tf')
import sys
import rospy
import math
import tf
from stage.srv import *
from stage.msg import Commands,Velocity

class MagnetCommunicator:

  def __init__(self):
    self.dt = rospy.get_param("Stage_Update_dt","0.010")

    self.tf_listener = tf.TransformListener()

    self.joy_sub = rospy.Subscriber("magnet/commands", Commands, self.commands_callback)
    self.vel_pub = rospy.Publisher("stage/command_velocity",Velocity)

    self.vel_values = Velocity()
    self.vel_scale_factor = 100     # mm/s

  def commands_callback(self,data):
    self.command_coordinates = data.header.frame_id
    self.radius_velocity = data.radius_velocity*self.vel_scale_factor
    self.tangent_velocity = data.tangent_velocity*self.vel_scale_factor
    self.x_velocity = data.x_velocity*self.vel_scale_factor
    self.y_velocity = data.y_velocity*self.vel_scale_factor

    if self.command_coordinates in 'Stage':
      self.vel_values.x_velocity = self.x_velocity
      self.vel_values.y_velocity = self.y_velocity
    elif self.command_coordinates in 'Plate':
      try:
        # (trans,rot) = self.tf_listener.lookupTransform('/Magnet', '/Plate', rospy.Time(0))
        (trans,rot) = self.tf_listener.lookupTransform('/Plate', '/Magnet', rospy.Time(0))
        x = trans[0]
        y = trans[1]
        theta = math.atan2(y,x)
        r = math.sqrt(x**2 + y**2)
        # rospy.logwarn("r = %s",str(r))
        # rospy.logwarn("theta = %s",str(theta))

        alpha = self.tangent_velocity*self.dt/(2*r)
        beta = self.tangent_velocity*self.dt/r
        try:
          chord_velocity = self.tangent_velocity*2*math.sin(beta/2)/beta
        except:
          chord_velocity = self.tangent_velocity

        x_vel_plate = self.radius_velocity*math.cos(theta) - chord_velocity*math.sin(theta+alpha)
        y_vel_plate = self.radius_velocity*math.sin(theta) + chord_velocity*math.cos(theta+alpha)
        # rospy.logwarn("x = %s",str(x))
        # rospy.logwarn("y = %s",str(y))
        # rospy.logwarn("r = %s",str(r))
        # rospy.logwarn("theta = %s",str(theta))
        # rospy.logwarn("r_velocity = %s",str(r_velocity))
        # rospy.logwarn("ang_velocity = %s",str(ang_velocity))
        # rospy.logwarn("x_vel_plate = %s",str(x_vel_plate))
        # rospy.logwarn("y_vel_plate = %s",str(y_vel_plate))
        self.vel_values.x_velocity = y_vel_plate
        self.vel_values.y_velocity = -x_vel_plate
      except (tf.LookupException, tf.ConnectivityException):
        pass

    self.vel_pub.publish(self.vel_values)

if __name__ == '__main__':
  rospy.init_node('MagnetCommunicator', anonymous=True)
  mc = MagnetCommunicator()

  while not rospy.is_shutdown():
    rospy.spin()
