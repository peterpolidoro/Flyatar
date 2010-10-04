#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_experiments')
import rospy
import smach
import smach_ros
import stage_action_server.msg
import time
import RobotMotionProfiles
from save_data.msg import SaveDataControls
import MonitorConditions
import random

class SaveDataControlsPublisher:
    def __init__(self):
        self.save_data_controls_pub = rospy.Publisher("SaveDataControls",SaveDataControls)
        self.save_data_controls = SaveDataControls()
        self.trial_count = 1

    def trial_count_increment(self):
        self.trial_count += 1

# Global Variables
SAVE_DATA_CONTROLS_PUBLISHER = SaveDataControlsPublisher()

# define state RecordData
class RecordData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state RECORD_DATA')
        global SAVE_DATA_CONTROLS_PUBLISHER
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.file_name_base = time.strftime("%Y_%m_%d_%H_%M_%S")
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.trial_number = int(SAVE_DATA_CONTROLS_PUBLISHER.trial_count)
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.rm_file = False
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.save_kinematics = True
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls)
        return 'succeeded'

# define state EraseData
class EraseData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state ERASE_DATA')
        global SAVE_DATA_CONTROLS_PUBLISHER
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.save_kinematics = False
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls)
        return 'succeeded'

# define state MonitorConditions
class MonitorConditions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.in_bounds_subscriber = MonitorConditions.InBoundsSubscriber()

    def execute(self, userdata):
        rospy.logwarn('Executing state MONITOR_CONDITIONS')
        while self.in_bounds_subscriber.robot_in_bounds and self.in_bounds_subscriber.fly_in_bounds:
            time.sleep(0.1)

        if self.in_bounds_subscriber.fly_in_bounds and (not self.in_bounds_subscriber.robot_in_bounds):
            return 'succeeded'
        else:
            return 'aborted'

# define state ControlRobot
class LogTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state LOG_TRIAL')
        global SAVE_DATA_CONTROLS_PUBLISHER
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.rm_file = False
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls.save_kinematics = False
        SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUBLISHER.save_data_controls)
        SAVE_DATA_CONTROLS_PUBLISHER.trial_count_increment()
        return 'succeeded'

class Trial():
    def __init__(self):
        # Create a SMACH state machine
        self.sm_trial = smach.StateMachine(['succeeded','aborted','preempted'])

        # Open the container
        with self.sm_trial:
            self.robot_motion_profile = RobotMotionProfiles.RobotMotionProfile()
            self.sm_robot_motion_profile = self.robot_motion_profile.sm_robot_motion_profile

            # Create the concurrent sub SMACH state machine
            self.sm_record_monitor_control = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                               default_outcome='aborted',
                                                               outcome_map={'succeeded':
                                                                            { 'RECORD_DATA':'succeeded',
                                                                              'MONITOR_CONDITIONS':'succeeded',
                                                                              'CONTROL_ROBOT':'succeeded'}})

            # Open the container
            with self.sm_record_monitor_control:
                # Add states to the container
                smach.Concurrence.add('RECORD_DATA', RecordData())
                smach.Concurrence.add('MONITOR_CONDITIONS', MonitorConditions())
                smach.Concurrence.add('CONTROL_ROBOT', self.sm_robot_motion_profile)

            # Add states to the container
            smach.StateMachine.add('RECORD_MONITOR_CONTROL', self.sm_record_monitor_control,
                                   transitions={'succeeded':'LOG_TRIAL',
                                                'aborted':'ERASE_DATA',
                                                'preempted':'ERASE_DATA'})

            smach.StateMachine.add('ERASE_DATA', EraseData(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('LOG_TRIAL', LogTrial(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

        # Create and start the introspection server
        self.sis = smach_ros.IntrospectionServer('sis_server_trial',
                                                 self.sm_trial,
                                                 '/SM_EXPERIMENT/SM_TRIAL')
        self.sis.start()

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_trial.execute()


if __name__ == '__main__':
    rospy.init_node('Trial')
    t = Trial()
    t.execute()
    t.sis.stop()
