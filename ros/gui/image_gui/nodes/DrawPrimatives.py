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
        self.draw_object.center = center

        self.circle = CvPrimatives.Circle((0,0),radius,color,thickness)
        self.draw_object.circle_list = [self.circle]

    def change_center(self,center):
        self.draw_object.center = center

    def change_radius(self,radius):
        self.circle.radius = radius

