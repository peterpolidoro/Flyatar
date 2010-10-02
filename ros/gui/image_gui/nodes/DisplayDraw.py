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

        self.circle_radius_min = 2

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
        self.draw_objects_on_image(self.draw_objects)

        cv.ShowImage("Display", self.im_display)
        cv.WaitKey(3)


    def draw_objects_on_image(self,draw_objects):
        for draw_object in draw_objects.draw_object_list:
            if (not draw_objects.hide_all) and (draw_object.show or draw_objects.show_all):
                self.draw_lines_on_image(draw_object.line_list,draw_object.object_center)
                self.draw_circles_on_image(draw_object.circle_list,draw_object.object_center)

    def draw_lines_on_image(self,line_list,object_center):
        for line in line_list:
            cv.Line(self.im_display,
                      ((object_center.x + line.point1.x),(object_center.y + line.point1.y)),
                      ((object_center.x + line.point2.x),(object_center.y + line.point2.y)),
                      cv.CV_RGB(line.color.red,line.color.green,line.color.blue),
                      line.thickness,
                      line.lineType,
                      line.shift)

    def draw_circles_on_image(self,circle_list,object_center):
        for circle in circle_list:
            if self.circle_radius_min <= circle.radius:
                cv.Circle(self.im_display,
                          ((object_center.x + circle.center.x),(object_center.y + circle.center.y)),
                          circle.radius,
                          cv.CV_RGB(circle.color.red,circle.color.green,circle.color.blue),
                          circle.thickness,
                          circle.lineType,
                          circle.shift)

    def draw_objects_callback(self,data):
        self.draw_objects = data

if __name__ == '__main__':
    rospy.init_node('ImageDisplayDraw')
    id = ImageDisplay()

    while not rospy.is_shutdown():
        rospy.spin()
