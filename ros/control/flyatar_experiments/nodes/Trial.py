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
import MonitorSystemState
import random
import numpy

class SaveDataControlsPublisher:
    def __init__(self):
        self.save_data_controls_pub = rospy.Publisher("SaveDataControls",SaveDataControls)
        self.save_data_controls = SaveDataControls()
        self.trial_count = 1

    def trial_count_increment(self):
        self.trial_count += 1

# Global Variables
SAVE_DATA_CONTROLS_PUB = SaveDataControlsPublisher()

# define state ChooseAngularVelocity
class ChooseAngularVelocity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.experiment_angular_velocity_max = rospy.get_param("experiment_angular_velocity_max") # rad/s
        self.experiment_angular_velocity_bin_count = rospy.get_param("experiment_angular_velocity_bin_count")
        self.angular_velocity_set_complete = set(numpy.linspace(-self.experiment_angular_velocity_max,
                                                                self.experiment_angular_velocity_max,
                                                                self.experiment_angular_velocity_bin_count,
                                                                True))
        self.angular_velocity_set = self.angular_velocity_set_complete

    def execute(self, userdata):
        rospy.logwarn('Executing state CHOOSE_ANGULAR_VELOCITY')
        try:
            angular_velocity = self.angular_velocity_set.pop()
            rospy.logwarn("angular velocity = %s" % (str(angular_velocity)))
        except (KeyError):
            self.angular_velocity_set = self.angular_velocity_set_complete

        return 'succeeded'

# define state RecordData
class RecordData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.protocol = "walking_protocol_type_1"

    def execute(self, userdata):
        rospy.logwarn('Executing state RECORD_DATA')
        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.save_data_controls.file_name_base = time.strftime("%Y_%m_%d_%H_%M_%S")
        SAVE_DATA_CONTROLS_PUB.save_data_controls.protocol = self.protocol
        SAVE_DATA_CONTROLS_PUB.save_data_controls.trial_number = int(SAVE_DATA_CONTROLS_PUB.trial_count)
        SAVE_DATA_CONTROLS_PUB.save_data_controls.angular_velocity_goal = 3.14
        SAVE_DATA_CONTROLS_PUB.save_data_controls.rm_file = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_kinematics = True
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_video = True
        SAVE_DATA_CONTROLS_PUB.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUB.save_data_controls)
        return 'succeeded'

# define state EraseData
class EraseData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state ERASE_DATA')
        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.save_data_controls.rm_file = True
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_kinematics = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_video = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUB.save_data_controls)
        return 'succeeded'

# define state MonitorConditions
class MonitorConditions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.in_bounds_sub = MonitorSystemState.InBoundsSubscriber()

    def execute(self, userdata):
        rospy.logwarn('Executing state MONITOR_CONDITIONS')

        while True:
            time.sleep(0.1)
        # while not self.in_bounds_sub.initialized:
        #     if self.preempt_requested():
        #         return 'preempted'
        #     time.sleep(0.1)

        # while True:
        #     if self.preempt_requested():
        #         return 'preempted'
        #     if not self.in_bounds_sub.in_bounds.fly_in_bounds:
        #         return 'aborted'
        #     time.sleep(0.1)

# define state ControlRobot
class LogTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.logwarn('Executing state LOG_TRIAL')
        global SAVE_DATA_CONTROLS_PUB
        SAVE_DATA_CONTROLS_PUB.save_data_controls.rm_file = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_kinematics = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls.save_video = False
        SAVE_DATA_CONTROLS_PUB.save_data_controls_pub.publish(SAVE_DATA_CONTROLS_PUB.save_data_controls)
        SAVE_DATA_CONTROLS_PUB.trial_count_increment()
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
                                                               # outcome_map={'succeeded':
                                                               #              { 'RECORD_DATA':'succeeded',
                                                               #                'CONTROL_ROBOT':'succeeded'}})
                                                                            # 'aborted':
                                                                            # { 'RECORD_DATA':'aborted',
                                                                            #   'MONITOR_CONDITIONS':'aborted',
                                                                            #   'CONTROL_ROBOT':'aborted'}})
                                                               # outcome_map={'succeeded':
                                                               #              { 'RECORD_DATA':'succeeded',
                                                               #                'MONITOR_CONDITIONS':'succeeded',
                                                               #                'CONTROL_ROBOT':'succeeded'}})
                                                               outcome_map={'succeeded':
                                                                            { 'RECORD_DATA':'succeeded',
                                                                              'MONITOR_CONDITIONS':'preempted',
                                                                              'CONTROL_ROBOT':'succeeded'}},
                                                               child_termination_cb = self.child_termination_callback)

            # Open the container
            with self.sm_record_monitor_control:
                # Add states to the container
                smach.Concurrence.add('RECORD_DATA', RecordData())
                smach.Concurrence.add('MONITOR_CONDITIONS', MonitorConditions())
                smach.Concurrence.add('CONTROL_ROBOT', self.sm_robot_motion_profile)

            # Add states to the container
            smach.StateMachine.add('CHOOSE_ANGULAR_VELOCITY', ChooseAngularVelocity(),
                                   transitions={'succeeded':'RECORD_MONITOR_CONTROL',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

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

    # gets called when ANY child state terminates
    def child_termination_callback(self,outcome_map):

        # rospy.logwarn("outcome_map = %s" % (str(outcome_map)))

        # terminate all running states if FOO finished with outcome 'outcome3'
        # if outcome_map['FOO'] == 'outcome3':
        #     # just keep running
        #     return False

        # terminate all running states if CONTROL_ROBOT finished
        if outcome_map['CONTROL_ROBOT'] == 'succeeded':
            return True

        # terminate all running states if MONITOR_CONDITIONS finished
        if outcome_map['MONITOR_CONDITIONS'] == 'aborted':
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    def execute(self):
        # Execute SMACH plan
        self.outcome = self.sm_trial.execute()


if __name__ == '__main__':
    rospy.init_node('Trial')
    t = Trial()
    t.execute()
    t.sis.stop()
