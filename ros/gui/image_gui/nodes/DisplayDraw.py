#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import sys
import rospy
import cv
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import image_gui.msg

class ImageDisplay:

    def __init__(self):
        self.image_sub = rospy.Subscriber("camera/image_rect", sensor_msgs.msg.Image, self.image_callback)

        cv.NamedWindow("Display",1)
        self.bridge = CvBridge()

        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.images_initialized = False

        self.draw_object_list_sub = rospy.Subscriber("DrawObjectList/image_rect", image_gui.msg.DrawObjectList, self.draw_object_list_callback)
        self.draw_object_list = image_gui.msg.DrawObjectList()

    def initialize_images(self,cv_image):
        self.im_display = cv.CreateImage(cv.GetSize(cv_image),cv.IPL_DEPTH_8U,3)
        self.images_initialized = True

    def image_callback(self,data):
        # Convert ROS image to OpenCV image
        try:
          cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
        except CvBridgeError, e:
          print e

        if not self.images_initialized:
            self.initialize_images(cv_image)

        cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)
        self.draw_objects(self.draw_object_list)

        cv.ShowImage("Display", self.im_display)
        cv.WaitKey(3)


    def draw_objects(self,draw_object_list):
        for draw_object in draw_object_list:
            self.draw_circles(draw_object.circle_list)

    def draw_circles(self,circle_list):
        for circle in circle_list:
            cv.Circle(self.im_display,
                      (circle.center.x,circle.center.x),
                      circle.radius,
                      cv.CV_RGB(circle.color.red,circle.color.green,circle.color.blue),
                      circle.thickness,
                      circle.lineType,
                      circle.shift)

    def draw_object_list_callback(self,data):
        self.draw_object_list = data

if __name__ == '__main__':
    rospy.init_node('ImageDisplayDraw')
    id = ImageDisplay()

    while not rospy.is_shutdown():
        rospy.spin()
