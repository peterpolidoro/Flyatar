#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import image_gui.msg

class DrawObjects:

    def __init__(self):
        self.color_max = 255
        self.draw_objects_pub = rospy.Publisher("DrawObjects/image_rect", image_gui.msg.DrawObjects)
        self.draw_objects = image_gui.msg.DrawObjects()
        self.circle = image_gui.msg.CvCircle()
        self.circle_list = []

        self.circle.center.x = 100
        self.circle.center.y = 100
        self.circle.radius = 50
        self.circle.color.red = self.color_max
        self.circle.color.green = 0
        self.circle.color.blue = 0
        self.circle.thickness = 2
        self.circle.lineType = 8
        self.circle.shift = 0
        self.circle_list.append(self.circle)
        self.draw_objects.circle_list = self.circle_list

        self.rate = rospy.Rate(10)

    def main(self):
        self.draw_objects_pub.publish(self.draw_objects)

if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()
    do.main()

    while not rospy.is_shutdown():
        do.main()
        do.rate.sleep()
        # rospy.spin()
