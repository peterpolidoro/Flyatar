#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg
import CvPrimatives

class CenteredCircle:
    def __init__(self,center,radius,color,thickness):
        self.draw_object = image_gui.msg.DrawObject()
        self.draw_object.object_center = center

        self.origin = CvPrimatives.Point(0,0)
        self.circle = CvPrimatives.Circle(self.origin.point,radius,color,thickness)
        self.draw_object.circle_list = [self.circle.circle]

    def change_center(self,center):
        self.draw_object.object_center = center

    def change_radius(self,radius):
        self.circle.change_radius(radius)

