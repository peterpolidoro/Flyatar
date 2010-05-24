#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('track_image_contours')
import sys
import rospy
from track_image_contours.msg import ContourInfo
from geometry_msgs.msg import PoseStamped

class ContourIdentifier:

  def __init__(self):

    # Listener/Subscribers
    self.contour_info_sub = rospy.Subscriber("ContourInfo",ContourInfo,self.contour_callback)

    # Broadcaster/Publishers
    self.robot_image_pose_pub = rospy.Publisher("RobotImagePose",PoseStamped)
    self.fly_image_pose_pub = rospy.Publisher("FlyImagePose",PoseStamped)

    # Pose
    self.robot_image_pose = PoseStamped()
    self.fly_image_pose = PoseStamped()

    # Robot Info
    self.robot_min_ecc = 1
    self.robot_max_ecc = 2
    self.robot_min_area = 2000
    self.robot_max_area = 6000

  def contour_callback(self,data):
    header = data.header
    x_list = data.x
    y_list = data.y
    theta_list = data.theta
    area_list = data.area
    ecc_list = data.ecc
    contour_count = min(len(x_list),len(y_list),len(theta_list),len(area_list),len(ecc_list))
    for contour in range(contour_count):
      x = x_list[contour]
      y = y_list[contour]
      theta = theta_list[contour]
      area = area_list[contour]
      ecc = ecc_list[contour]
      # Identify robot
      if ((self.robot_min_area < area) and (area < self.robot_max_area)) and ((self.robot_min_ecc < ecc) and (ecc < self.robot_max_ecc)):
        self.robot_image_pose.header = header
        self.robot_image_pose.pose.position.x = x
        self.robot_image_pose.pose.position.y = y
        self.robot_image_pose_pub.publish(self.robot_image_pose)
      elif contour_count == 2:
        self.fly_image_pose.header = header
        self.fly_image_pose.pose.position.x = x
        self.fly_image_pose.pose.position.y = y
        self.fly_image_pose_pub.publish(self.fly_image_pose)

def main(args):
  rospy.init_node('ContourIdentifier', anonymous=True)
  ci = ContourIdentifier()
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
