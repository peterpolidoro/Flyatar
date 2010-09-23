#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('stage_action_server')
import rospy
import actionlib
import stage_action_server.msg
import stage.srv
import tf

class StageUpdate:
  def __init__(self):
    self.initialized = False
    self.dt = rospy.get_param("Stage_Update_dt","0.010")

    self.rate = rospy.Rate(1/self.dt)

    self.tf_broadcaster = tf.TransformBroadcaster()

    self.stage_commands = stage.srv.Stage_StateRequest()
    self.update_position = False
    self.home = False

    rospy.wait_for_service('get_stage_state')
    try:
      self.get_stage_state = rospy.ServiceProxy('get_stage_state', stage.srv.Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('set_stage_position')
    try:
      self.set_stage_position = rospy.ServiceProxy('set_stage_position', stage.srv.Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    rospy.wait_for_service('home_stage')
    try:
      self.home_stage() = rospy.ServiceProxy('home_stage', stage.srv.Stage_State)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    self.initialized = True

  def update(self):
    if self.initialized:
      try:
        up = self.update_position
        h = self.home
        self.update_position = False
        self.home = False

        if up:
          self.response = self.set_stage_position(self.stage_commands)
        elif h:
          self.response = self.home_stage()
        else:
          self.response = self.get_stage_state()

        x = self.response.x
        y = self.response.y

        self.tf_broadcaster.sendTransform((x, y, 0),
                                          tf.transformations.quaternion_from_euler(0, 0, 0),
                                          rospy.Time.now(),
                                          "Magnet",
                                          "Stage")

      except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
        pass

class UpdateStagePositionAction(object):
  def __init__(self, name):
    self.initialized = False
    self.su = StageUpdate()

    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, stage_action_server.msg.UpdateStagePositionAction, execute_cb=self.execute_cb)

    # create messages that are used to publish feedback/result
    self.feedback = stage_action_server.msg.UpdateStagePositionFeedback()
    self.result   = stage_action_server.msg.UpdateStagePositionResult()

    self.goal_threshold = 0.1             # mm
    self.initialized = True

  def execute_cb(self, goal):
    if self.initialized:
      position_list_length = min(len(goal.x_position),len(goal.y_position))
      if (0 < position_list_length):
        self.su.update_position = True
        self.x_goal = goal.x_position[-1]
        self.y_goal = goal.y_position[-1]
      else:
        self.su.home = True
        self.x_goal = None
        self.y_goal = None

      self.su.stage_commands.x_position = goal.x_position
      self.su.stage_commands.y_position = goal.y_position
      self.su.stage_commands.velocity_magnitude = goal.velocity_magnitude

      # helper variables
      self.success = False

      # publish info to the console for the user
      # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

      # start executing the action
      while (not self.success):
        self.su.update()
        # rospy.logwarn("self.su.response.x = %s" % (str(self.su.response.x)))
        # rospy.logwarn("self.su.response.y = %s" % (str(self.su.response.y)))
        # rospy.logwarn("self.x_goal = %s" % (str(self.x_goal)))
        # rospy.logwarn("self.y_goal = %s" % (str(self.y_goal)))

        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          break
        if (self.x_goal is not None) and (self.y_goal is not None):
          if (not self.su.response.lookup_table_move_in_progress) and \
             self.su.response.all_motors_in_position and \
             self.su.response.motors_homed:
            if (abs(self.su.response.x - self.x_goal) < self.goal_threshold) and \
                   (abs(self.su.response.y - self.y_goal) < self.goal_threshold):
              self.success = True
            else:
              self._as.set_aborted()
              break
          else:
            self.feedback.x = self.su.response.x
            self.feedback.y = self.su.response.y
            self.feedback.x_velocity = self.su.response.x_velocity
            self.feedback.y_velocity = self.su.response.y_velocity
        else:
          if (not self.su.response.lookup_table_move_in_progress) and \
             self.su.response.all_motors_in_position and \
             self.su.response.motors_homed:
            self.success = True
            # if (abs(self.su.response.x - self.x_goal) < self.goal_threshold) and \
            #        (abs(self.su.response.y - self.y_goal) < self.goal_threshold):
            #   self.success = True
            # else:
            #   self._as.set_aborted()
            #   break

        self.su.rate.sleep()

      if self.success:
        self.result.x = self.su.response.x
        self.result.y = self.su.response.y
        self._as.set_succeeded(self.result)

if __name__ == '__main__':
  rospy.init_node('StageActionServer', anonymous=True)
  UpdateStagePositionAction(rospy.get_name())
  rospy.spin()
