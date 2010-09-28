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
        self.draw_objects_pub = rospy.Publisher("DrawObjects/image_rect", image_gui.msg.DrawObjects)
        self.draw_objects = image_gui.msg.DrawObjects()
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
        self.draw_objects.circle_list = self.circle_list

        rospy.logwarn("draw_objects = %s" % (str(self.draw_objects)))

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
