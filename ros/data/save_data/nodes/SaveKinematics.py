#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import time,os,subprocess
from save_data.msg import SaveDataControls, ExperimentConditions
from plate_tf.msg import RobotFlyKinematics

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveKinematics:
    def __init__(self):
        self.working_dir_base = os.path.expanduser("~/FlyatarData")
        chdir(self.working_dir_base)

        # Create new directory each day
        self.files_dir = time.strftime("%Y_%m_%d")
        self.working_dir = self.working_dir_base + "/" + self.files_dir
        chdir(self.working_dir)

        self.kinematics_sub = rospy.Subscriber("RobotFlyKinematics", RobotFlyKinematics, self.kinematics_callback)
        self.save_data_controls_sub = rospy.Subscriber("SaveDataControls", SaveDataControls, self.save_data_controls_callback)
        self.experiment_conditions_sub = rospy.Subscriber("ExperimentConditions", ExperimentConditions, self.experiment_conditions_callback)

        self.save_kinematics = False
        self.save_count = 0

        self.robot_move_commanded = False

        self.robot_width = rospy.get_param("robot_width","3.175") # mm
        self.robot_height = rospy.get_param("robot_height","3.175") # mm
        self.robot_visible = bool(rospy.get_param("robot_visible","true"))
        self.robot_paint = str(rospy.get_param("robot_paint","sharpie"))
        self.robot_pheromone = str(rospy.get_param("robot_pheromone","none"))
        self.trigger_angle = rospy.get_param("trigger_angle","1.5708") # rad

        self.header_column_titles = 'date_time protocol trial_number angular_velocity_goal robot_width robot_height robot_visible robot_paint robot_pheromone trigger_angle\n'
        self.data_column_titles = 'time robot_position_x robot_position_y robot_position_theta robot_velocity_x robot_velocity_y robot_velocity_theta robot_stopped fly_position_x fly_position_y fly_position_theta fly_velocity_x fly_velocity_y fly_velocity_theta fly_stopped robot_move_commanded\n'
        self.format_align = ">"
        self.format_sign = " "
        self.format_width = "7"
        self.format_precision = "2"
        self.format_type = "f"
        self.header_row_base = '{date_time:s} {protocol:s} {trial_number:>4d} {angular_velocity_goal:> 7.4f} {robot_width:>6.3f} {robot_height:>6.3f} {robot_visible:s} {robot_paint:s} {robot_pheromone:s} {trigger_angle:> 7.4f}\n\n'
        self.data_row_base = '{time:0.2f} {robot_position_x:{align}{sign}{width}.{precision}{type}} {robot_position_y:{align}{sign}{width}.{precision}{type}} {robot_position_theta:{align}{sign}{width}.{precision}{type}} {robot_velocity_x:{align}{sign}{width}.{precision}{type}} {robot_velocity_y:{align}{sign}{width}.{precision}{type}} {robot_velocity_theta:{align}{sign}{width}.{precision}{type}} {robot_stopped:{align}{sign}{width}d} {fly_position_x:{align}{sign}{width}.{precision}{type}} {fly_position_y:{align}{sign}{width}.{precision}{type}} {fly_position_theta:{align}{sign}{width}.{precision}{type}} {fly_velocity_x:{align}{sign}{width}.{precision}{type}} {fly_velocity_y:{align}{sign}{width}.{precision}{type}} {fly_velocity_theta:{align}{sign}{width}.{precision}{type}} {fly_stopped:{align}{sign}{width}d} {robot_move_commanded:{align}{sign}{width}d}\n'

    def experiment_conditions_callback(self,data):
        if data.robot_move_commanded:
            self.robot_move_commanded = True

    def save_data_controls_callback(self,data):
        if data.save_kinematics and (not self.save_kinematics):
            self.robot_move_commanded = False
            self.file_name = data.file_name_base + '.txt'
            self.fid = open(self.file_name, 'w')
            self.fid.write(self.header_column_titles)
            header_row = self.header_row_base.format(date_time = data.file_name_base,
                                                     protocol = data.protocol,
                                                     trial_number = data.trial_number,
                                                     angular_velocity_goal = data.angular_velocity_goal,
                                                     robot_width = self.robot_width,
                                                     robot_height = self.robot_height,
                                                     robot_visible = str(self.robot_visible),
                                                     robot_paint = self.robot_paint,
                                                     robot_pheromone = self.robot_pheromone,
                                                     trigger_angle = self.trigger_angle)
            self.fid.write(header_row)
            self.fid.write(self.data_column_titles)
            self.save_kinematics = data.save_kinematics
        elif (not data.save_kinematics) and self.save_kinematics:
            self.save_kinematics = data.save_kinematics
            self.save_count += 1
            self.fid.close()

        if data.rm_file:
            self.save_kinematics = False
            if os.path.exists(self.fid.name):
                if not self.fid.closed:
                    self.fid.close()
                os.remove(self.fid.name)

    def kinematics_callback(self,data):
        if self.save_kinematics:
            data_row = self.data_row_base.format(align = self.format_align,
                                                 sign = self.format_sign,
                                                 width = self.format_width,
                                                 precision = self.format_precision,
                                                 type = self.format_type,
                                                 time = data.header.stamp.to_sec(),
                                                 robot_position_x = data.robot_kinematics.position.x,
                                                 robot_position_y = data.robot_kinematics.position.y,
                                                 robot_position_theta = data.robot_kinematics.position.theta,
                                                 robot_velocity_x = data.robot_kinematics.velocity.x,
                                                 robot_velocity_y = data.robot_kinematics.velocity.y,
                                                 robot_velocity_theta = data.robot_kinematics.velocity.theta,
                                                 robot_stopped = int(data.robot_stopped.stopped),
                                                 fly_position_x = data.fly_kinematics.position.x,
                                                 fly_position_y = data.fly_kinematics.position.y,
                                                 fly_position_theta = data.fly_kinematics.position.theta,
                                                 fly_velocity_x = data.fly_kinematics.velocity.x,
                                                 fly_velocity_y = data.fly_kinematics.velocity.y,
                                                 fly_velocity_theta = data.fly_kinematics.velocity.theta,
                                                 fly_stopped = int(data.fly_stopped.stopped),
                                                 robot_move_commanded = int(self.robot_move_commanded))

            self.fid.write(data_row)

if __name__ == '__main__':
    rospy.init_node('SaveKinematics',log_level=rospy.INFO)
    sk = SaveKinematics()
    while not rospy.is_shutdown():
        rospy.spin()
