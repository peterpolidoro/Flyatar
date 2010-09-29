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
        rospy.logwarn('Executing state WAIT')
        time.sleep(5)
        return 'succeeded'

class Experiment():
    def __init__(self):
        # Create a SMACH state machine
        self.sm = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm:
            stage_goal = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal.x_position = [0]
            stage_goal.y_position = [0]
            stage_goal.velocity_magnitude = [50]

            stage_goal2 = stage_action_server.msg.UpdateStagePositionGoal()
            stage_goal2.x_position = [25]
            stage_goal2.y_position = [25]
            stage_goal2.velocity_magnitude = [50]

            # Add states to the container
            smach.StateMachine.add('GOTO_START',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal),
                                   transitions={'succeeded':'WAIT_FOR_FLY'})

            smach.StateMachine.add('WAIT_FOR_FLY', WaitForFly(),
                                   transitions={'succeeded':'GOTO_NEWPOSITION'})

            smach.StateMachine.add('GOTO_NEWPOSITION',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=stage_goal2),
                                   transitions={'succeeded':'succeeded'})

    def main(self):
        # Execute SMACH plan
        outcome = self.sm.execute()


if __name__ == '__main__':
    rospy.init_node('TestExperiment')
    e = Experiment()
    e.main()
