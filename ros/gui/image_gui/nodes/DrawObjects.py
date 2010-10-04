#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import copy
import image_gui.msg
import Colors
import CvPrimatives
import DrawPrimatives
import tf
from geometry_msgs.msg import PointStamped
import math

class DrawObjects:
    def __init__(self):
        self.initialized = False
        self.display_frame = "ImageRect"
        self.draw_objects_pub = rospy.Publisher("DrawObjects/image_rect", image_gui.msg.DrawObjects)
        self.draw_objects = image_gui.msg.DrawObjects()

        self.dt = rospy.get_param("Draw_Update_dt","0.010")
        self.rate = rospy.Rate(1/self.dt)

        self.in_bounds_radius_plate = rospy.get_param('in_bounds_radius',1)
        self.axis_length_plate = 4

        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service('plate_to_camera')
        try:
            self.plate_to_camera = rospy.ServiceProxy('plate_to_camera', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        self.plate_camera_origin = PointStamped()
        self.plate_camera_origin.header.frame_id = "Camera"
        self.plate_camera_point = PointStamped()
        self.plate_camera_point.header.frame_id = "Camera"
        Xsrc = [0,self.in_bounds_radius_plate,self.axis_length_plate]
        Ysrc = [0,0,0]
        response = self.plate_to_camera(Xsrc,Ysrc)
        x0 = self.plate_origin_camera.point.x = response.Xdst[0]
        y0 = self.plate_origin_camera.point.y = response.Ydst[0]
        x1 = self.plate_point_camera.point.x = response.Xdst[1]
        y1 = self.plate_point_camera.point.y = response.Ydst[1]
        x2 = response.Xdst[1]
        y2 = response.Ydst[1]
        self.in_bounds_radius_camera = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        self.axis_length_camera = math.sqrt((x2-x0)**2 + (y2-y0)**2)

        self.plate_image_origin = PointStamped()
        self.plate_image_origin.header.frame_id = "PlateImage"
        self.plate_image_origin.point.x = 0
        self.plate_image_origin.point.y = 0
        self.plate_image_origin.point.z = 0
        self.robot_image_origin = PointStamped()
        self.robot_image_origin.header.frame_id = "RobotImage"
        self.robot_image_origin.point.x = 0
        self.robot_image_origin.point.y = 0
        self.robot_image_origin.point.z = 0
        self.robot_image_origin = PointStamped()
        self.robot_image_origin.header.frame_id = "RobotImage"
        self.robot_image_origin.point.x = 0
        self.robot_image_origin.point.y = 0
        self.robot_image_origin.point.z = 0



        self.colors = Colors.Colors()

        self.origin = CvPrimatives.Point(0,0)

        self.tf_listener.waitForTransform(self.display_frame, "PlateImage", rospy.Time(0), rospy.Duration(4.0))
        self.plate_image_origin_display_frame = self.tf_listener.transformPoint(self.display_frame,
                                                                                self.plate_image_origin)

        self.plate_origin_primatives = CvPrimatives.Point(self.plate_image_origin_display_frame.point.x,
                                                          self.plate_image_origin_display_frame.point.y)

        self.plate_origin_marker = DrawPrimatives.Axes(self.plate_origin_primatives.point)

        self.robot_marker = DrawPrimatives.CenteredCircle(self.origin.point,self.axis_length_camera,self.colors.blue,2)

        self.in_bounds_marker = DrawPrimatives.CenteredCircle(self.plate_origin_primatives.point,self.in_bounds_radius_camera,self.colors.yellow,1)

        self.draw_objects.draw_object_list = [self.plate_origin_marker.draw_object,
                                              self.robot_marker.draw_object,
                                              self.in_bounds_marker.draw_object]
        self.draw_objects.show_all = False
        self.draw_objects.hide_all = False
        self.initialized = True

    def update(self):
        if self.initialized:
            try:
                self.robot_image_origin_display_frame = self.tf_listener.transformPoint(self.display_frame,
                                                                                        self.robot_image_origin)
                self.robot_marker.change_center(CvPrimatives.Point(self.robot_image_origin_display_frame.point.x,
                                                                   self.robot_image_origin_display_frame.point.y).point)
            except (tf.LookupException, tf.ConnectivityException):
                pass

            self.draw_objects_pub.publish(self.draw_objects)

if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()

    while not rospy.is_shutdown():
        do.update()
        do.rate.sleep()
