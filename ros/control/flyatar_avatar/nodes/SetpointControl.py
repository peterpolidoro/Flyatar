#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('flyatar_avatar')
import sys
import rospy
import math
import tf
import cv
import numpy
from plate_tf.srv import *
from stage.srv import *
from stage.msg import StageCommands,Setpoint
from joystick_commands.msg import JoystickCommands
from geometry_msgs.msg import PointStamped

class SetpointControl:

    def __init__(self):
        self.initialized = False
        self.control_dt = rospy.get_param("Control_Update_dt")

        self.tf_listener = tf.TransformListener()

        self.joy_sub = rospy.Subscriber("Joystick/Commands", JoystickCommands, self.joystick_commands_callback)
        self.sc_pub = rospy.Publisher("StageCommands",StageCommands)
        self.setpoint_pub = rospy.Publisher("setpoint",Setpoint)

        self.sc_ok_to_publish = False

        self.stage_commands = StageCommands()
        self.stage_commands.position_control = False
        self.robot_velocity_max = rospy.get_param("robot_velocity_max",100) # mm/s
        self.vel_vector_plate = numpy.array([[0],[0],[0],[1]])
        self.vel_vector_robot = numpy.array([[0],[0],[0],[1]])

        self.control_frame = "Plate"
        self.start_frame = "Plate"

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0
        self.robot_origin_position = PointStamped()
        self.robot_origin_position.header.frame_id = "Plate"
        self.setpoint_origin = PointStamped()
        self.setpoint_origin.header.frame_id = "Fly"
        self.setpoint_origin.point.x = 0
        self.setpoint_origin.point.y = 0
        self.setpoint_origin.point.z = 0
        self.setpoint_origin_position = PointStamped()
        self.setpoint_origin_position.header.frame_id = "Plate"
        self.setpoint_position = PointStamped()
        self.setpoint_position.header.frame_id = "Plate"
        self.setpoint_position.point.x = 0
        self.setpoint_position.point.y = 0
        self.setpoint_position.point.z = 0

        # self.setpoint_position_initialized = False
        # while not self.setpoint_position_initialized:
        #     try:
        #         self.setpoint_position = self.tf_listener.transformPoint("Stage",self.setpoint_position)
        #         self.setpoint_position_initialized = True
        #     except (tf.LookupException, tf.ConnectivityException):
        #         pass

        self.moving_to_start = False
        self.moving_to_setpoint = False

        self.setpoint = Setpoint()
        self.setpoint.header.frame_id = self.control_frame
        self.setpoint.radius = 20
        self.setpoint.theta = 0
        self.inc_radius = 1
        self.inc_theta = 0.05
        self.setpoint_radius_max = 80
        self.setpoint_radius_min = 0

        self.rate = rospy.Rate(1/self.control_dt)
        self.gain_radius = rospy.get_param("gain_radius","4")
        self.gain_theta = rospy.get_param("gain_theta","40")

        self.radius_velocity = 0
        self.tangent_velocity = 0
        self.tracking = False

        rospy.wait_for_service('plate_to_stage')
        try:
            self.plate_to_stage = rospy.ServiceProxy('plate_to_stage', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Xsrc = [0,10,15.4,-34.1]
        # Ysrc = [0,10,-15.3,-65.9]
        # rospy.logwarn("plate_points_x = %s" % (str(Xsrc)))
        # rospy.logwarn("plate_points_y = %s" % (str(Ysrc)))
        # response = self.plate_to_stage(Xsrc,Ysrc)
        # rospy.logwarn("stage_points_x = %s" % (str(response.Xdst)))
        # rospy.logwarn("stage_points_y = %s" % (str(response.Ydst)))

        self.initialized = True

    def circle_dist(self,setpoint,angle):
        diff1 = setpoint - angle
        if angle < setpoint:
            diff2 = setpoint - 2*math.pi - angle
        else:
            diff2 = 2*math.pi - angle + setpoint
        abs_min = min(abs(diff1),abs(diff2))
        if abs_min == abs(diff1):
            return diff1
        else:
            return diff2

    def vel_vector_convert(self,vel_vector,frame):
        (trans,q) = self.tf_listener.lookupTransform("Stage",frame,rospy.Time(0))
        rot_matrix = tf.transformations.quaternion_matrix(q)
        vel_vector_stage = numpy.dot(rot_matrix,vel_vector)
        x_vel_stage = vel_vector_stage[0]
        y_vel_stage = vel_vector_stage[1]
        return [x_vel_stage],[y_vel_stage]

    def find_robot_origin_position(self):
        robot_origin_position_acquired = False
        while not robot_origin_position_acquired:
            try:
                self.robot_origin_position = self.tf_listener.transformPoint("Plate",self.robot_origin)
                robot_origin_position_acquired = True
            except (tf.LookupException, tf.ConnectivityException):
                rospy.logwarn("Error finding robot_origin_position_stage!")

    def find_setpoint_origin_position(self):
        setpoint_origin_position_acquired = False
        while not setpoint_origin_position_acquired:
            try:
                self.setpoint_origin_position = self.tf_listener.transformPoint("Plate",self.setpoint_origin)
                setpoint_origin_position_acquired = True
            except (tf.LookupException, tf.ConnectivityException):
                rospy.logwarn("Error finding robot_origin_position_stage!")

    def find_velocity_from_position(self,pos_x,pos_y,vel_mag):
        point_count = min(len(pos_x),len(pos_y))
        vel_x = []
        vel_y = []
        vel_mag = abs(vel_mag)          # Just to be sure
        for point_n in range(point_count):
            if point_n == 0:
                vel_x.append(math.sqrt(2)*vel_mag)
                vel_y.append(math.sqrt(2)*vel_mag)
            else:
                delta_x = abs(pos_x[point_n] - pos_x[point_n - 1])
                delta_y = abs(pos_y[point_n] - pos_y[point_n - 1])
                alpha = math.sqrt((vel_mag**2)/(delta_x**2 + delta_y**2))
                vel_x.append(alpha*delta_x)
                vel_y.append(alpha*delta_y)
        return vel_x,vel_y

    def set_stage_commands_from_position_points(self,plate_points_x,plate_points_y,vel_mag):
        response = self.plate_to_stage(plate_points_x,plate_points_y)
        stage_points_x = response.Xdst
        stage_points_y = response.Ydst
        stage_velocity_x,stage_velocity_y = self.find_velocity_from_position(stage_points_x,stage_points_y,vel_mag)
        self.stage_commands.x_position = stage_points_x
        self.stage_commands.y_position = stage_points_y
        self.stage_commands.x_velocity = stage_velocity_x
        self.stage_commands.y_velocity = stage_velocity_y

        # rospy.logwarn("plate_points_x = %s" % (str(plate_points_x)))
        # rospy.logwarn("plate_points_y = %s" % (str(plate_points_y)))
        # rospy.logwarn("stage_points_x = %s" % (str(stage_points_x)))
        # rospy.logwarn("stage_points_y = %s" % (str(stage_points_y)))

    def set_path_to_start(self,vel_mag):
        self.find_robot_origin_position()
        plate_points_x = [self.robot_origin_position.point.x,0]
        plate_points_y = [self.robot_origin_position.point.y,0]
        self.set_stage_commands_from_position_points(plate_points_x,plate_points_y,vel_mag)
        # self.set_position_velocity_point(0,0,self.start_frame,vel_mag)

    def set_path_to_setpoint(self,vel_mag):
        self.find_robot_origin_position()
        self.find_setpoint_origin_position()
        x_ro = self.robot_origin_position.point.x
        y_ro = self.robot_origin_position.point.x
        x_so =  self.setpoint_origin_position.point.x
        y_so =  self.setpoint_origin_position.point.y
        dx = x_ro - x_so
        dy = y_ro - y_so
        dr = math.sqrt(dx**2 + dy**2)
        dx_norm = dx/dr
        dy_norm = dy/dr
        xi = x_so + dx_norm*self.setpoint.radius
        yi = y_so + dy_norm*self.setpoint.radius
        plate_points_x = [x_ro,xi]
        plate_points_y = [y_ro,yi]
        self.set_stage_commands_from_position_points(plate_points_x,plate_points_y,vel_mag)

    # def set_position_velocity_point(self,x_target,y_target,frame_target,vel_mag):
    #     self.setpoint_position.header.frame_id = frame_target
    #     self.setpoint_position.point.x = x_target
    #     self.setpoint_position.point.y = y_target

    #     target_acquired = False
    #     while not target_acquired:
    #         try:
    #             setpoint_position_stage = self.tf_listener.transformPoint("Stage",self.setpoint_position)
    #             target_acquired = True
    #         except (tf.LookupException, tf.ConnectivityException):
    #             rospy.logwarn("Error finding setpoint_position_stage!")

    #     self.stage_commands.x_position = [robot_origin_position_stage.point.x,setpoint_position_stage.point.x]
    #     self.stage_commands.y_position = [robot_origin_position_stage.point.y,setpoint_position_stage.point.y]

    #     delta_x = abs(setpoint_position_stage.point.x - robot_origin_position_stage.point.x)
    #     delta_y = abs(setpoint_position_stage.point.y - robot_origin_position_stage.point.y)
    #     alpha = math.sqrt((vel_mag**2)/(delta_x**2 + delta_y**2))
    #     self.stage_commands.x_velocity = [vel_mag,alpha*delta_x]
    #     self.stage_commands.y_velocity = [vel_mag,alpha*delta_y]

    def joystick_commands_callback(self,data):
        if self.initialized:
            self.control_frame = data.header.frame_id
            self.setpoint.header.frame_id = self.control_frame
            self.setpoint_origin.header.frame_id = self.control_frame
            self.setpoint.radius += self.inc_radius*data.radius_inc
            if self.setpoint.radius < self.setpoint_radius_min:
                self.setpoint.radius = self.setpoint_radius_min
            elif self.setpoint_radius_max < self.setpoint.radius:
                self.setpoint.radius = self.setpoint_radius_max
            self.setpoint.theta += self.inc_theta*data.theta_inc
            self.setpoint.theta = math.fmod(self.setpoint.theta,2*math.pi)
            if self.setpoint.theta < 0:
                self.setpoint.theta = 2*math.pi - self.setpoint.theta
            self.tracking = data.tracking
            self.setpoint_pub.publish(self.setpoint)

            if not self.tracking:
                if data.start:
                    if not self.moving_to_start:
                        self.moving_to_start = True
                        self.stage_commands.position_control = True
                        self.set_path_to_start(self.robot_velocity_max/2)
                        self.sc_ok_to_publish = True
                    else:
                        self.sc_ok_to_publish = False
                else:
                    self.sc_ok_to_publish = True
                    if self.moving_to_start:
                        self.moving_to_start = False
                    self.stage_commands.position_control = False
                    self.radius_velocity = data.radius_velocity*self.robot_velocity_max
                    self.tangent_velocity = data.tangent_velocity*self.robot_velocity_max
                    self.vel_vector_plate[0,0] = data.x_velocity*self.robot_velocity_max
                    self.vel_vector_plate[1,0] = data.y_velocity*self.robot_velocity_max

                    try:
                        (self.stage_commands.x_velocity,self.stage_commands.y_velocity) = self.vel_vector_convert(self.vel_vector_plate,"Plate")
                    except (tf.LookupException, tf.ConnectivityException):
                        self.stage_commands.x_velocity = [self.vel_vector_plate[0,0]]
                        self.stage_commands.y_velocity = [-self.vel_vector_plate[1,0]]

                if self.sc_ok_to_publish:
                    self.sc_pub.publish(self.stage_commands)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.tracking:
                if not self.moving_to_setpoint:
                    self.moving_to_setpoint = True
                    self.stage_commands.position_control = True
                    self.set_path_to_setpoint(self.robot_velocity_max/4)
                    self.sc_ok_to_publish = True
                else:
                    self.sc_ok_to_publish = False

                if self.sc_ok_to_publish:
                    self.sc_pub.publish(self.stage_commands)

                # try:
                    # self.gain_radius = rospy.get_param("gain_radius")
                    # self.gain_theta = rospy.get_param("gain_theta")

                    # (self.stage_commands.x_velocity,self.stage_commands.y_velocity) = self.vel_vector_convert(self.vel_vector_plate,"Plate")

                    # (trans,q) = self.tf_listener.lookupTransform(self.control_frame,'/Robot',rospy.Time(0))
                    # x = trans[0]
                    # y = trans[1]
                    # theta = math.atan2(y,x)
                    # radius = math.sqrt(x**2 + y**2)

                    # # rospy.logwarn("self.setpoint.radius = \n%s",str(self.setpoint.radius))
                    # # rospy.logwarn("radius = \n%s",str(radius))
                    # # rospy.logwarn("self.radius_velocity = \n%s",str(self.radius_velocity))

                    # self.gain_radius = self.gain_radius + (6/math.pi)*abs(self.circle_dist(self.setpoint.theta,theta))
                    # self.radius_velocity = self.gain_radius*(self.setpoint.radius - radius)
                    # # rospy.logwarn("self.gain_radius = \n%s",str(self.gain_radius))
                    # self.tangent_velocity = self.gain_theta*self.circle_dist(self.setpoint.theta,theta)

                    # rot_matrix = tf.transformations.rotation_matrix(theta, (0,0,1))
                    # self.vel_vector_robot[0,0] = self.radius_velocity
                    # self.vel_vector_robot[1,0] = self.tangent_velocity
                    # vel_vector_plate = numpy.dot(rot_matrix,self.vel_vector_robot)
                    # self.vel_vector_plate[0,0] = vel_vector_plate[0]
                    # self.vel_vector_plate[1,0] = vel_vector_plate[1]

                    # (x_vel_stage,y_vel_stage) = self.vel_vector_convert(self.vel_vector_plate,"Plate")
                    # self.stage_commands.x_velocity += x_vel_stage
                    # self.stage_commands.y_velocity += y_vel_stage

                #     self.sc_pub.publish(self.stage_commands)
                # except (tf.LookupException, tf.ConnectivityException):
                #     pass

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SetpointControl')
    sc = SetpointControl()
    sc.control_loop()
