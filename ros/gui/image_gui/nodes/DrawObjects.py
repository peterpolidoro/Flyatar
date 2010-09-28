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
# from geometry_msgs.msg import PoseStamped

class DrawObjects:

    def __init__(self):
        self.draw_object_list_pub = rospy.Publisher("DrawObjectList/image_rect", image_gui.msg.DrawObjectList)
        self.draw_object_list = image_gui.msg.DrawObjectList()

        self.colors = colors.Colors()
        # self.robot_image_pose_sub = rospy.Subscriber('ImagePose/Robot',PoseStamped,self.handle_robot_image_pose)
        self.origin = CvPrimatives.Point(0,0)
        self.marker1 = DrawPrimatives.CenteredCircle(self.origin.point,50,self.colors.red,2)
        self.marker2 = DrawPrimatives.CenteredCircle(self.origin.point,25,self.colors.blue,-1)
        self.marker2.change_center((100,100))
        self.marker2.change_center((200,200))

        self.draw_object_list.draw_object_list = [self.marker1.draw_object,self.marker2.draw_object]

        self.rate = rospy.Rate(10)

    def main(self):
        self.draw_object_list_pub.publish(self.draw_object_list)

if __name__ == '__main__':
    rospy.init_node('DrawObjects')
    do = DrawObjects()
    do.main()

    while not rospy.is_shutdown():
        do.main()
        do.rate.sleep()
        # rospy.spin()
