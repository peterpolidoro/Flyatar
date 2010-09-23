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

    self.initialized = True

  def update(self):
    if self.initialized:
      try:
        up = self.update_position
        self.update_position = False

        if up:
          self.response = self.set_stage_position(self.stage_commands)
          # rospy.logwarn("set_stage_position()")
        else:
          self.response = self.get_stage_state()
          # rospy.logwarn("get_stage_state()")

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
    su = StageUpdate()

    rospy.logwarn("Creating SimpleActionServer")
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, stage_action_server.msg.UpdateStagePositionAction, execute_cb=self.execute_cb)
    rospy.logwarn("Created SimpleActionServer...")

    # create messages that are used to publish feedback/result
    self.feedback = stage_action_server.msg.UpdateStagePositionFeedback()
    self.result   = stage_action_server.msg.UpdateStagePositionResult()

    self.goal_threshold = 1             # mm
    self.initialized = True

  def execute_cb(self, goal):
    rospy.logwarn("In execute_cb...")
    if self.initialized:
      position_list_length = min(len(goal.x_position),len(goal.y_position))
      if (0 < position_list_length):
        su.update_position = True
        self.x_goal = goal.x_position[-1]
        self.y_goal = goal.y_position[-1]
      else:
        su.update_position = False
        self.x_goal = None
        self.y_goal = None

      su.stage_commands.x_position = goal.x_position
      su.stage_commands.y_position = goal.y_position
      su.stage_commands.velocity_magnitude = goal.velocity_magnitude

      # helper variables
      self.success = False

      # publish info to the console for the user
      # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

      # start executing the action
      while not self.success:
        su.update()
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          break
        if (self.x_goal is not None) and (self.y_goal is not None):
          if (not su.response.lookup_table_move_in_progress) and \
             su.response.all_motors_in_position and \
             su.response.motors_homed and \
             (abs(su.response.x - self.x_goal) < self.goal_threshold) and \
             (abs(su.response.y - self.y_goal) < self.goal_threshold):
            self.success = True
          else:
            self.feedback.x = su.response.x
            self.feedback.y = su.response.y
            self.feedback.x_velocity = su.response.x_velocity
            self.feedback.y_velocity = su.response.y_velocity
            self.feedback.motors_homed = su.response.motors_homed
        else:
          self.success = True

        su.rate.sleep()

      if success:
        self.result.x = su.response.x
        self.result.y = su.response.y
        self.result.motors_homed = su.response.motors_homed
        self._as.set_succeeded(self.result)

if __name__ == '__main__':
  rospy.init_node('StageActionServer', anonymous=True)
  UpdateStagePositionAction(rospy.get_name())
  rospy.spin()
