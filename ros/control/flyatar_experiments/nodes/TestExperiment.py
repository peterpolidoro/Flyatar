#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import tf
from plate_tf.srv import *
from plate_tf.msg import StopState, InBoundsState

# define state ExperimentStart
class ExperimentStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'




class Experiment():
    def __init__(self):
        self.tf_listener = tf.TransformListener()

    def main():
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1':'BAR',
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome2':'FOO'})

        # Execute SMACH plan
        outcome = sm.execute()


if __name__ == '__main__':
    rospy.init_node('TestExperiment')
    e = Experiment()
    e.main()
