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
    def __init__(self,center,radius,color,thickness,lineType=8,shift=0):
        self.circle = image_gui.msg.CvCircle()
        self.circle.center = center
        self.circle.radius = radius
        self.circle.color = color
        self.circle.thickness = thickness
        self.circle.lineType = lineType
        self.circle.shift = shift

    def change_center(self,center):
        self.circle.center = center

    def change_radius(self,radius):
        self.circle.radius = radius

class Line:
    def __init__(self,pt1,pt2,color,thickness,lineType=8,shift=0):
        self.line = image_gui.msg.CvLine()
        self.line.pt1 = pt1
        self.line.pt2 = pt2
        self.line.color = color
        self.line.thickness = thickness
        self.line.lineType = lineType
        self.line.shift = shift

    def change_pt1(self,pt):
        self.line.pt1 = pt

    def change_pt2(self,pt):
        self.line.pt2 = pt
