#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import copy
import image_gui.msg
import colors
import CvPrimatives
import DrawPrimatives
from geometry_msgs.msg import PoseStamped

class DrawObjects:

    def __init__(self):
        self.draw_object_list_pub = rospy.Publisher("DrawObjectList/image_rect", image_gui.msg.DrawObjectList)
        self.draw_object_list = image_gui.msg.DrawObjectList()

        self.colors = colors.Colors()

        self.origin = CvPrimatives.Point(0,0)
        self.robot_marker = DrawPrimatives.CenteredCircle(self.origin.point,25,self.colors.blue,2)

        self.robot_image_pose_sub = rospy.Subscriber('ImagePose/Robot',PoseStamped,self.handle_robot_image_pose)

        self.plate_origin_marker = DrawPrimatives.Axes(CvPrimatives.Point(300,300).point)

        self.draw_object_list.draw_object_list = [self.plate_origin_marker.draw_object,
                                                  self.robot_marker.draw_object]
        self.draw_object_list.show_all = False
        self.draw_object_list.hide_all = False

        self.rate = rospy.Rate(10)

    def handle_robot_image_pose(self,data):
        self.robot_marker_x += 1
        self.robot_marker_y += 2
        self.robot_marker.change_center(CvPrimatives.Point(self.robot_marker_x,self.robot_marker_y).point)
        self.draw_object_list_pub.publish(self.draw_object_list)

if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()

    while not rospy.is_shutdown():
        rospy.spin()
