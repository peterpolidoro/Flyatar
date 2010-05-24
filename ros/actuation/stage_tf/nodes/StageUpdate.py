#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_tf')
import rospy
import tf
from stage.msg import Velocity
from stage.srv import *

class StageUpdate:

  def __init__(self):
    self.dt = rospy.get_param("Stage_Update_dt","0.010")

    self.rate = rospy.Rate(1/self.dt)

    self.tf_broadcaster = tf.TransformBroadcaster()

    self.vel_commands = Velocity_StateRequest()
    self.update_velocity = False

    self.vel_sub = rospy.Subscriber("stage/command_velocity", Velocity, self.velocity_callback)

    rospy.wait_for_service('get_stage_state')
    try:
        self.get_stage_state = rospy.ServiceProxy('get_stage_state', Velocity_State)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_velocity')
    try:
        self.set_stage_velocity = rospy.ServiceProxy('set_stage_velocity', Velocity_State)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def velocity_callback(self,data):
      self.vel_commands.x_velocity = data.x_velocity
      self.vel_commands.y_velocity = data.y_velocity
      self.update_velocity = True

  def updater(self):
    while not rospy.is_shutdown():
      try:
        if not self.update_velocity:
          response = self.get_stage_state()
          x = response.x
          y = response.y
        else:
          response = self.set_stage_velocity(self.vel_commands)
          x = response.x
          y = response.y
          self.update_velocity = False

        self.tf_broadcaster.sendTransform((x, y, 0),
                                          tf.transformations.quaternion_from_euler(0, 0, 0),
                                          rospy.Time.now(),
                                          "Magnet",
                                          "Stage")
      except (rospy.service.ServiceException):
        pass

      self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('StageUpdate', anonymous=True)
  su = StageUpdate()
  su.updater()
