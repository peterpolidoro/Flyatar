#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg

class Point:
    def __init__(self,x,y):
        self.point = image_gui.msg.CvPoint()
        self.point.x = x
        self.point.y = y

    def change_center(self,x,y):
        self.point.x = x
        self.point.y = y

class Circle:
    def __init__(self,center,radius,color,thickness):
        self.circle = image_gui.msg.CvCircle()
        self.circle.center = center
        self.circle.radius = radius
        self.circle.color = color
        self.circle.thickness = thickness
        self.circle.lineType = 8
        self.circle.shift = 0

    def change_center(self,center):
        self.circle.center = center

    def change_radius(self,radius):
        self.circle.radius = radius

