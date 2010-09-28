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

        self.color_max = 255
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.images_initialized = False

        self.draw_objects_sub = rospy.Subscriber("DrawObjects/image_rect", image_gui.msg.DrawObjects, self.draw_objects_callback)
        self.draw_objects = image_gui.msg.DrawObjects()

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
        self.draw_circles(self.draw_objects.circle_list)

        cv.ShowImage("Display", self.im_display)
        cv.WaitKey(3)


    def draw_circles(self,circle_list):
        for circle_N in range(len(circle_list)):
            cv.Circle(self.im_display,
                      circle_list[circle_N].center,
                      circle_list[circle_N].radius,
                      cv.CV_RGB(circle_list[circle_N].color.red,circle_list[circle_N].color.green,circle_list[circle_N].color.blue),
                      circle_list[circle_N].thickness,
                      circle_list[circle_N].lineType,
                      circle_list[circle_N].shift)

    def draw_objects_callback(self,data):
        self.draw_objects = data

if __name__ == '__main__':
    rospy.init_node('ImageDisplayDraw')
    id = ImageDisplay()

    while not rospy.is_shutdown():
        rospy.spin()
