#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import time,os,subprocess
from save_data.msg import SaveDataControls
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
        self.files_dir = time.strftime("%Y_%m_%d")
        self.working_dir = self.working_dir_base + "/" + self.files_dir
        chdir(self.working_dir)

        self.kinematics_sub = rospy.Subscriber("RobotFlyKinematics", RobotFlyKinematics, self.kinematics_callback)
        self.save_data_controls_sub = rospy.Subscriber("SaveDataControls", SaveDataControls, self.save_data_controls_callback)

        self.save_kinematics = False
        self.save_count = 0

        self.header = 'Wait Straight Experiment\ndistance_units: mm\nangle_units: rad\n\n'
        self.column_titles = 'time robot_position_x robot_position_y robot_position_theta robot_velocity_x robot_velocity_y robot_velocity_theta robot_stopped fly_position_x fly_position_y fly_position_theta fly_velocity_x fly_velocity_y fly_velocity_theta fly_stopped\n'
        self.format_align = ">"
        self.format_sign = " "
        self.format_width = "7"
        self.format_precision = "2"
        self.format_type = "f"
        self.data_row_base = '{time:0.2f} {robot_position_x:{align}{sign}{width}.{precision}{type}} {robot_position_y:{align}{sign}{width}.{precision}{type}} {robot_position_theta:{align}{sign}{width}.{precision}{type}} {robot_velocity_x:{align}{sign}{width}.{precision}{type}} {robot_velocity_y:{align}{sign}{width}.{precision}{type}} {robot_velocity_theta:{align}{sign}{width}.{precision}{type}} {robot_stopped:{align}{sign}{width}d} {fly_position_x:{align}{sign}{width}.{precision}{type}} {fly_position_y:{align}{sign}{width}.{precision}{type}} {fly_position_theta:{align}{sign}{width}.{precision}{type}} {fly_velocity_x:{align}{sign}{width}.{precision}{type}} {fly_velocity_y:{align}{sign}{width}.{precision}{type}} {fly_velocity_theta:{align}{sign}{width}.{precision}{type}} {fly_stopped:{align}{sign}{width}d}\n'

    def save_data_controls_callback(self,data):
        if data.save_kinematics and (not self.save_kinematics):
            # self.file_name = "trial{num:04d}\n".format(num=self.save_count)
            self.file_name = time.strftime("%Y_%m_%d_%H_%M_%S.txt")
            self.fid = open(self.file_name, 'w')
            self.fid.write(self.header)
            self.fid.write(self.column_titles)
            self.save_kinematics = data.save_kinematics
        elif (not data.save_kinematics) and self.save_kinematics:
            self.save_kinematics = data.save_kinematics
            self.save_count += 1
            self.fid.close()

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
                                                 fly_stopped = int(data.fly_stopped.stopped))

            self.fid.write(data_row)

if __name__ == '__main__':
    rospy.init_node('SaveKinematics',log_level=rospy.INFO)
    sk = SaveKinematics()
    while not rospy.is_shutdown():
        rospy.spin()
