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

save_data_controls_pub = rospy.Publisher("SaveDataControls",SaveDataControls)
save_data_controls = SaveDataControls()

# define state RecordData
class RecordData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RECORD_DATA')
        save_data_controls.save_kinematics = True
        save_data_controls_pub.publish(save_data_controls)
        return 'succeeded'

# define state EraseData
class EraseData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ERASE_DATA')
        time.sleep(3)
        return 'succeeded'

# define state MonitorConditions
class MonitorConditions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MONITOR_CONDITIONS')
        time.sleep(3)
        return 'succeeded'

# define state ControlRobot
class LogTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LOG_TRIAL')
        save_data_controls.save_kinematics = False
        save_data_controls_pub.publish(save_data_controls)
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
