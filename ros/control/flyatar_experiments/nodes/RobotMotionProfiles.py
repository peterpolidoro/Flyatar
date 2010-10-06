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

# Global Variables
FLY_VIEW_SUB = MonitorSystemState.FlyViewSubscriber()
KINEMATICS_SUB = MonitorSystemState.KinematicsSubscriber()

# define state WaitForTriggerCondition
class WaitForTriggerCondition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_TRIGGER_CONDITION')

        while not FLY_VIEW_SUB.initialized:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        rospy.logwarn("Waiting for robot to be in front of fly")
        while not FLY_VIEW_SUB.fly_view.robot_in_front_of_fly:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        rospy.logwarn("Waiting for fly to be walking")
        while KINEMATICS_SUB.kinematics.fly_stopped:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        rospy.logwarn("Waiting for robot to be behind fly")
        while FLY_VIEW_SUB.fly_view.robot_in_front_of_fly:
            if self.preempt_requested():
                return 'preempted'
            time.sleep(0.1)

        return 'succeeded'

# define state CalculateMove
class CalculateMove(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['succeeded','aborted','preempted'],
                             output_keys = ['x_position_list','y_position_list','velocity_magnitude_list'])
        self.in_bounds_radius = rospy.get_param("in_bounds_radius") # mm
        self.move_distance = self.in_bounds_radius + 5
        self.experiment_velocity_max = rospy.get_param("experiment_velocity_max") # mm/s

    def execute(self, userdata):
        rospy.logwarn('Executing state CALCULATE_MOVE')
        fly_vx = KINEMATICS_SUB.kinematics.fly_kinematics.velocity.x
        fly_vy = KINEMATICS_SUB.kinematics.fly_kinematics.velocity.y
        fly_vmag = numpy.linalg.norm([fly_vx,fly_vy])
        vx_norm = fly_vx/fly_vmag
        vy_norm = fly_vy/fly_vmag
        rospy.logwarn("fly_vx = %s" % (str(fly_vx)))
        rospy.logwarn("fly_vy = %s" % (str(fly_vy)))
        move_direction = random.choice([-1,1])
        move_x_position = move_direction*vx_norm*self.move_distance
        move_y_position = move_direction*vy_norm*self.move_distance
        rospy.logwarn("move_x_position = %s" % (str(move_x_position)))
        rospy.logwarn("move_y_position = %s" % (str(move_y_position)))
        rospy.logwarn("velocity_magnitude = %s" % (str(self.experiment_velocity_max)))
        userdata.x_position_list = [move_x_position]
        userdata.y_position_list = [move_y_position]
        userdata.velocity_magnitude_list = [self.experiment_velocity_max]
        return 'succeeded'

class RobotMotionProfile():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_robot_motion_profile = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_robot_motion_profile:
            stage_goal = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal.x_position = [25]
            stage_goal.y_position = [25]
            stage_goal.velocity_magnitude = [50]

            # Add states to the container
            smach.StateMachine.add('WAIT_FOR_TRIGGER_CONDITION', WaitForTriggerCondition(),
                                   transitions={'succeeded':'CALCULATE_MOVE',
                                                'preempted':'preempted',
                                                'aborted':'aborted'})

            smach.StateMachine.add('CALCULATE_MOVE', CalculateMove(),
                                   transitions={'succeeded':'MOVE_ROBOT',
                                                'preempted':'preempted',
                                                'aborted':'aborted'})

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
