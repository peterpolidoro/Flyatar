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

# Global Variables
FLY_STATE = MonitorSystemState.FlyState()

# define state WaitForTriggerCondition
class WaitForTriggerCondition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state WAIT_FOR_TRIGGER_CONDITION')
        while True:
            if FLY_STATE.initialized:
                rospy.logwarn("robot_in_front_of_fly = %s" % (str(FLY_STATE.robot_in_front_of_fly)))
            time.sleep(0.2)
        return 'succeeded'

# define state CalculateMove
class CalculateMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state CALCULATE_MOVE')
        time.sleep(5)
        return 'succeeded'

# define state MoveRobot
class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state MOVE_ROBOT')
        time.sleep(5)
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
                                   transitions={'succeeded':'CALCULATE_MOVE'})

            smach.StateMachine.add('CALCULATE_MOVE', CalculateMove(),
                                   transitions={'succeeded':'MOVE_ROBOT'})

            smach.StateMachine.add('MOVE_ROBOT',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal),
                                   transitions={'succeeded':'succeeded'})

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
