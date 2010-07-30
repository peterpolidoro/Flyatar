#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_tf')
import rospy
import tf
from stage.msg import StageCommands,StageState
from stage.srv import *

class StageUpdate:

  def __init__(self):
    self.initialized = False
    self.dt = rospy.get_param("Stage_Update_dt","0.010")

    self.rate = rospy.Rate(1/self.dt)

    self.tf_broadcaster = tf.TransformBroadcaster()

    self.stage_commands = Stage_StateRequest()
    self.update_position = False
    self.update_velocity = False
    self.lookup_table_correct = False

    self.updated = True

    self.sc_sub = rospy.Subscriber("Stage/Commands", StageCommands, self.stage_commands_callback)
    self.ss_pub = rospy.Publisher("Stage/State",StageState)
    self.ss = StageState()

    rospy.wait_for_service('get_stage_state')
    try:
      self.get_stage_state = rospy.ServiceProxy('get_stage_state', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_velocity')
    try:
      self.set_stage_velocity = rospy.ServiceProxy('set_stage_velocity', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_position')
    try:
      self.set_stage_position = rospy.ServiceProxy('set_stage_position', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('stage_lookup_table_correct')
    try:
      self.stage_lookup_table_correct = rospy.ServiceProxy('stage_lookup_table_correct', Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    # rospy.wait_for_service('stage_lookup_table_move')
    # try:
    #   self.stage_lookup_table_move = rospy.ServiceProxy('stage_lookup_table_move', Stage_State)
    # except rospy.ServiceException, e:
    #   print "Service call failed: %s"%e

    self.initialized = True

  def stage_commands_callback(self,data):
    if self.initialized and self.updated:
      if data.position_control:
        rospy.logwarn ("data.position_control = %s" % (str(data.position_control)))
        self.update_position = True
        self.update_velocity = False
        self.lookup_table_correct = False
      elif data.velocity_control:
        # rospy.logwarn ("data.velocity_control = %s" % (str(data.velocity_control)))
        self.update_position = False
        self.update_velocity = True
        self.lookup_table_correct = False
      elif data.lookup_table_correct:
        rospy.logwarn ("data.lookup_table_correct = %s" % (str(data.lookup_table_correct)))
        self.update_position = False
        self.update_velocity = False
        self.lookup_table_correct = True
      else:
        self.update_position = False
        self.update_velocity = False
        self.lookup_table_correct = False

      self.stage_commands.x_position = data.x_position
      self.stage_commands.y_position = data.y_position
      self.stage_commands.x_velocity = data.x_velocity
      self.stage_commands.y_velocity = data.y_velocity

  def updater(self):
    while not rospy.is_shutdown():
      if self.initialized:
        try:
          self.updated = False
          if self.update_position:
            self.update_position = False
            response = self.set_stage_position(self.stage_commands)
            # response = self.stage_lookup_table_move(self.stage_commands)
            # rospy.logwarn("set_stage_position()")
            x = response.x
            y = response.y
            self.ss.all_motors_in_position = response.all_motors_in_position
            self.ss.lookup_table_move_complete = response.lookup_table_move_complete
            self.update_position = False
          elif self.update_velocity:
            self.update_velocity = False
            response = self.set_stage_velocity(self.stage_commands)
            # rospy.logwarn("set_stage_velocity()")
            x = response.x
            y = response.y
            self.ss.all_motors_in_position = response.all_motors_in_position
            self.ss.lookup_table_move_complete = response.lookup_table_move_complete
            self.update_velocity = False
          elif self.lookup_table_correct:
            self.lookup_table_correct = False
            response = self.stage_lookup_table_correct(self.stage_commands)
            x = response.x
            y = response.y
            self.ss.all_motors_in_position = response.all_motors_in_position
            self.ss.lookup_table_move_complete = response.lookup_table_move_complete
          else:
            response = self.get_stage_state()
            # rospy.logwarn("get_stage_state()")
            x = response.x
            y = response.y
            self.ss.all_motors_in_position = response.all_motors_in_position
            self.ss.lookup_table_move_complete = response.lookup_table_move_complete

          self.ss_pub.publish(self.ss)
          self.tf_broadcaster.sendTransform((x, y, 0),
                                            tf.transformations.quaternion_from_euler(0, 0, 0),
                                            rospy.Time.now(),
                                            "Magnet",
                                            "Stage")
        except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
          pass

        self.updated = True

        self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('StageUpdate', anonymous=True)
  su = StageUpdate()
  su.updater()
