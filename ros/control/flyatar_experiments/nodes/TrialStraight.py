#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time

# define state Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        time.sleep(5)
        return 'succeeded'

class Trial():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_trial = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_trial:
            stage_goal1 = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal1.x_position = [25]
            stage_goal1.y_position = [25]
            stage_goal1.velocity_magnitude = [50]

            stage_goal2 = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal2.x_position = [-25]
            stage_goal2.y_position = [15]
            stage_goal2.velocity_magnitude = [25]

            # Add states to the container
            smach.StateMachine.add('GOTO_GOAL1',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal1),
                                   transitions={'succeeded':'GOTO_GOAL2'})

            smach.StateMachine.add('GOTO_GOAL2',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal2),
                                   transitions={'succeeded':'succeeded'})


    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_trial.execute()


if __name__ == '__main__':
    rospy.init_node('TrialStraight')
    t = Trial()
    t.execute()
