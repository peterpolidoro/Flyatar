#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg
import colors
import CvPrimatives

colors = colors.Colors()

class Axes:
    def __init__(self,center):
        self.draw_object = image_gui.msg.DrawObject()
        self.draw_object.object_center = center
        self.axis_length = 20

        self.origin_point = CvPrimatives.Point(0,0)
        self.x_axis_point = CvPrimatives.Point(self.axis_length,0)
        self.y_axis_point = CvPrimatives.Point(0,self.axis_length)

        self.x_axis_line = CvPrimatives.Line(self.origin_point.point,
                                             self.x_axis_point.point,
                                             colors.red,
                                             2)

        self.y_axis_line = CvPrimatives.Line(self.origin_point.point,
                                             self.y_axis_point.point,
                                             colors.green,
                                             2)

        self.draw_object.line_list = [self.x_axis_line,self.y_axis_line]
        self.draw_object.show = True

    def change_center(self,center):
        self.draw_object.object_center = center

class CenteredCircle:
    def __init__(self,center,radius,color,thickness):
        self.draw_object = image_gui.msg.DrawObject()
        self.draw_object.object_center = center

        self.origin = CvPrimatives.Point(0,0)
        self.circle = CvPrimatives.Circle(self.origin.point,radius,color,thickness)
        self.draw_object.circle_list = [self.circle.circle]
        self.draw_object.show = True

    def change_center(self,center):
        self.draw_object.object_center = center

    def change_radius(self,radius):
        self.circle.change_radius(radius)

