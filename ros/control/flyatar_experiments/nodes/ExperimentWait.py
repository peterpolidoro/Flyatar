#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time
import Trial
from plate_tf.msg import InBounds

# define state WaitForFly
class WaitForFly(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.in_bounds_sub = rospy.Subscriber('InBounds',InBounds,self.in_bounds_callback)
        self.fly_in_bounds = False

    def in_bounds_callback(self,data):
        self.fly_in_bounds = data.fly_in_bounds

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_FOR_FLY')
        while not self.fly_in_bounds:
            time.sleep(0.1)

        return 'succeeded'

class Experiment():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_experiment = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_experiment:
            self.start_position = stage_action_server.msg.UpdateStagePositionGoal()
            self.start_position.x_position = [0]
            self.start_position.y_position = [0]
            self.start_position.velocity_magnitude = [50]

            self.trial = Trial.Trial()
            self.sm_trial = self.trial.sm_trial

            # Add states to the container
            smach.StateMachine.add('GOTO_START',
                                   smach_ros.SimpleActionState('StageActionServer',
                                                               stage_action_server.msg.UpdateStagePositionAction,
                                                               goal=self.start_position),
                                   transitions={'succeeded':'WAIT_FOR_FLY',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('WAIT_FOR_FLY', WaitForFly(),
                                   transitions={'succeeded':'RUN_TRIAL',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

            smach.StateMachine.add('RUN_TRIAL',
                                   self.sm_trial,
                                   transitions={'succeeded':'GOTO_START',
                                                'aborted':'GOTO_START',
                                                'preempted':'GOTO_START'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_experiment',
                                                 self.sm_experiment,
                                                 '/SM_EXPERIMENT')
        self.sis.start()

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_experiment.execute()


if __name__ == '__main__':
    rospy.init_node('ExperimentWait')
    e = Experiment()
    e.execute()
    e.sis.stop()
