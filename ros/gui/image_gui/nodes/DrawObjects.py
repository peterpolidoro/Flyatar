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

class DrawObjects:
    def __init__(self):
        self.initialized = False
        self.display_frame = "ImageRect"
        self.draw_objects_pub = rospy.Publisher("DrawObjects/image_rect", image_gui.msg.DrawObjects)
        self.draw_objects = image_gui.msg.DrawObjects()

        self.dt = rospy.get_param("Draw_Update_dt","0.010")
        self.rate = rospy.Rate(1/self.dt)

        self.tf_listener = tf.TransformListener()
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

        self.colors = Colors.Colors()

        self.origin = CvPrimatives.Point(0,0)
        self.robot_marker = DrawPrimatives.CenteredCircle(self.origin.point,25,self.colors.blue,2)

        self.tf_listener.waitForTransform(self.display_frame, "PlateImage", rospy.Time(0), rospy.Duration(4.0))
        self.plate_image_origin_display_frame = self.tf_listener.transformPoint(self.display_frame,
                                                                                self.plate_image_origin)

        self.plate_origin_marker = DrawPrimatives.Axes(CvPrimatives.Point(self.plate_image_origin_display_frame.point.x,
                                                                          self.plate_image_origin_display_frame.point.y).point)

        self.draw_objects.draw_object_list = [self.plate_origin_marker.draw_object,
                                              self.robot_marker.draw_object]
        self.draw_objects.show_all = False
        self.draw_objects.hide_all = False
        self.initialized = True

    def update(self):
        if self.initialized:
            try:
                self.robot_image_origin_display_frame = self.tf_listener.transformPoint(self.display_frame,
                                                                                        self.robot_image_origin)
            except (tf.LookupException, tf.ConnectivityException):
                pass

            self.robot_marker.change_center(CvPrimatives.Point(self.robot_image_origin_display_frame.point.x,
                                                               self.robot_image_origin_display_frame.point.y).point)
            self.draw_objects_pub.publish(self.draw_objects)

if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()

    while not rospy.is_shutdown():
        do.update()
        do.rate.sleep()
