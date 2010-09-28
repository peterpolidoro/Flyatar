#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('image_gui')
import rospy
import copy
import image_gui.msg
import colors
# from geometry_msgs.msg import PoseStamped

class DrawObjects:

    def __init__(self):
        self.draw_object_list_pub = rospy.Publisher("DrawObjectList/image_rect", image_gui.msg.DrawObjectList)
        self.draw_object_list = image_gui.msg.DrawObjectList()
        self.draw_object = image_gui.msg.DrawObject()
        self.circle = image_gui.msg.CvCircle()
        self.circle_list = []

        self.colors = colors.Colors()
        # self.robot_image_pose_sub = rospy.Subscriber('ImagePose/Robot',PoseStamped,self.handle_robot_image_pose)


        self.circle.center.x = 100
        self.circle.center.y = 100
        self.circle.radius = 50
        self.circle.color = self.colors.cyan
        self.circle.thickness = 2
        self.circle.lineType = 8
        self.circle.shift = 0
        self.circle_list.append(copy.deepcopy(self.circle))
        self.circle.center.x = 400
        self.circle.color = self.colors.yellow
        self.circle_list.append(copy.deepcopy(self.circle))
        self.draw_object.circle_list = self.circle_list
        self.draw_object_list.draw_object_list = [self.draw_object]

        rospy.logwarn("draw_objects = %s" % (str(self.draw_objects)))

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
