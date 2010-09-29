#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time
import TrialStraight as Trial

# define state Wait
class WaitForFly(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_FLY')
        time.sleep(5)
        return 'succeeded'

class Experiment():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_experiment = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_experiment:
            plate_origin_goal = stage_action_server.msg.UpdateStagePositionGoal()
            plate_origin_goal.x_position = [0]
            plate_origin_goal.y_position = [0]
            plate_origin_goal.velocity_magnitude = [50]

            self.Trial = Trial.Trial()
            self.sm_trial = self.Trial.sm_trial

            # Add states to the container
            smach.StateMachine.add('GOTO_START',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=plate_origin_goal),
                                   transitions={'succeeded':'WAIT_FOR_FLY'})

            smach.StateMachine.add('WAIT_FOR_FLY', WaitForFly(),
                                   transitions={'succeeded':'RUN_TRIAL'})

            smach.StateMachine.add('RUN_TRIAL',
                                   self.sm_trial,
                                   transitions={'succeeded':'GOTO_START'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_experiment', self.sm_experiment, '/SM_EXPERIMENT')
        self.sis.start()

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_experiment.execute()


if __name__ == '__main__':
    rospy.init_node('ExperimentWait')
    e = Experiment()
    e.execute()
    e.sis.stop()
