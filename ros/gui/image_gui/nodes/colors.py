#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg

class Colors:
    def __init__(self):
        self.color_max = 255

        self.red = image_gui.msg.CvColor()
        self.red.red = self.color_max
        self.red.green = 0
        self.red.blue = 0
        self.green = image_gui.msg.CvColor()
        self.green.red = 0
        self.green.green = self.color_max
        self.green.blue = 0
        self.blue = image_gui.msg.CvColor()
        self.blue.red = 0
        self.blue.green = 0
        self.blue.blue = self.color_max
        self.purple = image_gui.msg.CvColor()
        self.purple.red = self.color_max
        self.purple.green = 0
        self.purple.blue = self.color_max
