#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time
import MonitorSystemState
import random
import numpy
import math
from save_data.msg import ExperimentConditions

# Global Variables
FLY_VIEW_SUB = MonitorSystemState.FlyViewSubscriber()
KINEMATICS_SUB = MonitorSystemState.KinematicsSubscriber()

# define state WaitForTriggerCondition
class WaitForTriggerCondition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
        self.trigger_angle = abs(rospy.get_param("trigger_angle","1.5708"))
        self.sleep_time = 0.1
        self.timeout = 30
        self.watchdog = 0

    # Recalculate to make sure angle range is from -pi to pi
    def find_robot_angle(self):
        x = FLY_VIEW_SUB.fly_view.robot_position_x
        y = FLY_VIEW_SUB.fly_view.robot_position_y
        angle = math.atan2(y,x)
        return angle

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_TRIGGER_CONDITION')

        self.watchdog = 0
        while not FLY_VIEW_SUB.initialized:
            if self.preempt_requested():
                return 'preempted'
            if self.timeout < self.watchdog:
                return 'aborted'
            time.sleep(self.sleep_time)
            self.watchdog += self.sleep_time

        rospy.logwarn("Waiting for robot to be in front of fly")
        self.watchdog = 0
        self.robot_angle = self.find_robot_angle()
        while (self.trigger_angle < abs(self.robot_angle)):
            if self.preempt_requested():
                return 'preempted'
            if self.timeout < self.watchdog:
                return 'aborted'
            time.sleep(self.sleep_time)
            self.watchdog += self.sleep_time
            self.robot_angle = self.find_robot_angle()

        # rospy.logwarn("Waiting for fly to be walking")
        # while KINEMATICS_SUB.kinematics.fly_stopped:
        #     if self.preempt_requested():
        #         return 'preempted'
        #     time.sleep(0.1)

        rospy.logwarn("Waiting for trigger")
        self.watchdog = 0
        self.robot_angle = self.find_robot_angle()
        while (abs(self.robot_angle) < self.trigger_angle):
            if self.preempt_requested():
                return 'preempted'
            if self.timeout < self.watchdog:
                return 'aborted'
            time.sleep(self.sleep_time)
            self.watchdog += self.sleep_time
            self.robot_angle = self.find_robot_angle()

        return 'succeeded'

# define state WaitForZeroVelocityMoveEndCondition
# This State should not be necessary, but something strange is preempting MonitorConditions...
class WaitForZeroVelocityMoveEndCondition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        self.in_bounds_sub = MonitorSystemState.InBoundsSubscriber()
        self.sleep_time = 0.1
        self.timeout = 30
        self.watchdog = 0

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_ZERO_VELOCITY_MOVE_END_CONDITION')

        self.watchdog = 0
        while not self.in_bounds_sub.initialized:
            if self.preempt_requested():
                return 'preempted'
            if self.timeout < self.watchdog:
                return 'aborted'
            time.sleep(self.sleep_time)
            self.watchdog += self.sleep_time

        self.time_start = rospy.Time.now().to_sec()
        while True:
            if self.preempt_requested():
                rospy.logwarn("WaitForZeroVelocityMoveEndCondition preempted")
                return 'preempted'
            if not self.in_bounds_sub.in_bounds.fly_in_bounds:
                rospy.logwarn("WaitForZeroVelocityMoveEndCondition fly_left_bounds")
                return 'succeeded'
            time.sleep(self.sleep_time)
            self.time_now = rospy.Time.now().to_sec()
            if self.timeout < abs(self.time_now - self.time_start):
                rospy.logwarn("WaitForZeroVelocityMoveEndCondition timeout")
                return 'succeeded'

# define state CalculateMove
class CalculateMove(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['succeeded','zero_velocity_move','aborted'],
                             input_keys = ['angular_velocity_input'],
                             output_keys = ['x_position_list','y_position_list','velocity_magnitude_list'])

        self.experiment_conditions_pub = rospy.Publisher("ExperimentConditions",ExperimentConditions)
        self.experiment_conditions = ExperimentConditions()

        self.in_bounds_radius = rospy.get_param("in_bounds_radius") # mm
        self.start_position_x = rospy.get_param("start_position_x")
        self.start_position_y = rospy.get_param("start_position_y")
        self.move_distance = rospy.get_param("move_distance")
        self.experiment_linear_velocity_min = rospy.get_param("experiment_linear_velocity_min",'5') # mm/s
        self.experiment_linear_velocity_max = rospy.get_param("experiment_linear_velocity_max",'100') # mm/s
        self.trigger_angle = abs(rospy.get_param("trigger_angle","1.5708"))
        self.angular_velocity_min = 0.01 # rad/s

    def execute(self, userdata):
        rospy.logwarn('Executing state CALCULATE_MOVE')

        rospy.logwarn("CALCULATE_MOVE angular_velocity = %s" % (str(userdata.angular_velocity_input)))

        if abs(userdata.angular_velocity_input) < self.angular_velocity_min:
            self.experiment_conditions.robot_move_commanded = True
            self.experiment_conditions_pub.publish(self.experiment_conditions)
            return 'zero_velocity_move'

        fly_px = KINEMATICS_SUB.kinematics.fly_kinematics.position.x
        fly_py = KINEMATICS_SUB.kinematics.fly_kinematics.position.y
        robot_px = KINEMATICS_SUB.kinematics.robot_kinematics.position.x
        robot_py = KINEMATICS_SUB.kinematics.robot_kinematics.position.y
        fly_to_robot_px = robot_px - fly_px
        fly_to_robot_py = robot_py - fly_py

        robot_vx_a = -fly_to_robot_py
        robot_vy_a = fly_to_robot_px
        robot_vx_b = fly_to_robot_py
        robot_vy_b = -fly_to_robot_px

        fly_vx = KINEMATICS_SUB.kinematics.fly_kinematics.velocity.x
        fly_vy = KINEMATICS_SUB.kinematics.fly_kinematics.velocity.y

        dot_a = numpy.dot([fly_vx,fly_vy],[robot_vx_a,robot_vy_a])
        dot_b = numpy.dot([fly_vx,fly_vy],[robot_vx_b,robot_vy_b])

        angular_velocity = userdata.angular_velocity_input

        if 0 < (dot_a*angular_velocity):
            robot_vx = robot_vx_a
            robot_vy = robot_vy_a
        elif 0 < (dot_b*angular_velocity):
            robot_vx = robot_vx_b
            robot_vy = robot_vy_b
        else:
            rospy.logwarn("Could not choose robot velocity direction!")
            return 'aborted'

        rospy.logwarn("fly_px = %s" % (str(fly_px)))
        rospy.logwarn("fly_py = %s" % (str(fly_py)))
        rospy.logwarn("robot_px = %s" % (str(robot_px)))
        rospy.logwarn("robot_py = %s" % (str(robot_py)))
        rospy.logwarn("fly_to_robot_px = %s" % (str(fly_to_robot_px)))
        rospy.logwarn("fly_to_robot_py = %s" % (str(fly_to_robot_py)))
        rospy.logwarn("fly_vx = %s" % (str(fly_vx)))
        rospy.logwarn("fly_vy = %s" % (str(fly_vy)))
        rospy.logwarn("robot_vx = %s" % (str(robot_vx)))
        rospy.logwarn("robot_vy = %s" % (str(robot_vy)))

        robot_vmag = numpy.linalg.norm([robot_vx,robot_vy])
        vx_norm = robot_vx/robot_vmag
        vy_norm = robot_vy/robot_vmag
        # move_direction = math.copysign(1,angular_velocity)
        # move_direction = random.choice([-1,1])
        # move_x_position = move_direction*vx_norm*self.move_distance + robot_px
        # move_y_position = move_direction*vy_norm*self.move_distance + robot_py
        move_x_position = vx_norm*self.move_distance + robot_px
        move_y_position = vy_norm*self.move_distance + robot_py
        rospy.logwarn("move_x_position = %s" % (str(move_x_position)))
        rospy.logwarn("move_y_position = %s" % (str(move_y_position)))
        userdata.x_position_list = [move_x_position]
        userdata.y_position_list = [move_y_position]
        robot_distance = FLY_VIEW_SUB.fly_view.robot_distance
        rospy.logwarn("robot_distance = %s" % (str(robot_distance)))
        linear_velocity = abs(userdata.angular_velocity_input)*abs(robot_distance)
        if linear_velocity < self.experiment_linear_velocity_min:
            linear_velocity = self.experiment_linear_velocity_min
        if self.experiment_linear_velocity_max < linear_velocity:
            linear_velocity = self.experiment_linear_velocity_max
        userdata.velocity_magnitude_list = [linear_velocity]
        rospy.logwarn("velocity_magnitude = %s" % (str(linear_velocity)))

        self.experiment_conditions.robot_move_commanded = True
        self.experiment_conditions_pub.publish(self.experiment_conditions)

        return 'succeeded'

class RobotMotionProfile():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_robot_motion_profile = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'],
                                                          input_keys = ['angular_velocity_rmp'])
        # self.sm_robot_motion_profile.userdata.angular_velocity_rmp = 0

        # Open the container
        with self.sm_robot_motion_profile:
            # stage_goal = stage_action_server.msg.UpdateStagePositionGoal()
            # stage_goal.x_position = [25]
            # stage_goal.y_position = [25]
            # stage_goal.velocity_magnitude = [50]

            # Add states to the container
            smach.StateMachine.add('WAIT_FOR_TRIGGER_CONDITION', WaitForTriggerCondition(),
                                   transitions={'succeeded':'CALCULATE_MOVE',
                                                'preempted':'preempted',
                                                'aborted':'aborted'})

            smach.StateMachine.add('CALCULATE_MOVE', CalculateMove(),
                                   transitions={'succeeded':'MOVE_ROBOT',
                                                'zero_velocity_move':'WAIT_FOR_ZERO_VELOCITY_MOVE_END_CONDITION',
                                                'aborted':'aborted'},
                                   remapping={'angular_velocity_input':'angular_velocity_rmp'})

            smach.StateMachine.add('MOVE_ROBOT',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal_slots=['x_position',
                                                                           'y_position',
                                                                           'velocity_magnitude']),
                                                               # goal=stage_goal),
                                   transitions={'succeeded':'succeeded',
                                                'preempted':'preempted',
                                                'aborted':'aborted'},
                                   remapping={'x_position':'x_position_list',
                                              'y_position':'y_position_list',
                                              'velocity_magnitude':'velocity_magnitude_list'})

            smach.StateMachine.add('WAIT_FOR_ZERO_VELOCITY_MOVE_END_CONDITION', WaitForZeroVelocityMoveEndCondition(),
                                   transitions={'succeeded':'succeeded',
                                                'preempted':'preempted'})



        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_robot_motion_profile',
                                                 self.sm_robot_motion_profile,
                                                 '/SM_EXPERIMENT/SM_TRIAL/SM_ROBOT_MOTION_PROFILE')
        self.sis.start()

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_robot_motion_profile.execute()


if __name__ == '__main__':
    rospy.init_node('RobotMotionProfiles')
    rmp = RobotMotionProfile()
    rmp.execute()
    rmp.sis.stop()
