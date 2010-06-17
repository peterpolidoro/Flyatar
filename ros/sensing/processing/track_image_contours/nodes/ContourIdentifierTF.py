#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('track_image_contours')
import sys
import rospy
from track_image_contours.msg import ContourInfo
from geometry_msgs.msg import PoseStamped,PointStamped
from plate_tf.srv import *
import tf,math

class ContourIdentifier:

  def __init__(self):
    self.initialized = False

    # Listener/Subscribers
    self.contour_info_sub = rospy.Subscriber("ContourInfo",ContourInfo,self.contour_callback)
    self.tf_listener = tf.TransformListener()

    # Broadcaster/Publishers
    self.robot_image_pose_pub = rospy.Publisher("RobotImagePose",PoseStamped)
    self.fly_image_pose_pub = rospy.Publisher("FlyImagePose",PoseStamped)

    # Pose
    self.robot_image_pose = PoseStamped()
    self.fly_image_pose = PoseStamped()

    # Points
    self.magnet_origin = PointStamped()
    self.magnet_origin.header.frame_id = "Magnet"
    self.magnet_origin.point.x = 0
    self.magnet_origin.point.y = 0
    self.magnet_origin_camera = PointStamped()
    self.magnet_origin_camera.header.frame_id = "Camera"
    self.magnet_origin_camera.point.x = 0
    self.magnet_origin_camera.point.y = 0

    # Robot Info
    self.robot_min_ecc = rospy.get_param("robot_min_ecc",0.5)
    self.robot_max_ecc = rospy.get_param("robot_max_ecc",3)
    self.robot_min_area = rospy.get_param("robot_min_area",1000)
    self.robot_max_area = rospy.get_param("robot_max_area",10000)
    # self.robot_min_ecc = 0.5
    # self.robot_max_ecc = 3
    # self.robot_min_area = 1000
    # self.robot_max_area = 10000

    rospy.wait_for_service('plate_to_camera')
    try:
      self.plate_to_camera = rospy.ServiceProxy('plate_to_camera', PlateCameraConversion)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    self.initialized = True

  def contour_callback(self,data):
    if self.initialized:
      header = data.header
      x_list = data.x
      y_list = data.y
      theta_list = data.theta
      area_list = data.area
      ecc_list = data.ecc
      contour_count = min(len(x_list),len(y_list),len(theta_list),len(area_list),len(ecc_list))
      dist_list = []
      robot_list = []
      dist_list = []

      try:
        magnet_origin_plate = self.tf_listener.transformPoint("Plate",self.magnet_origin)
        Xsrc = [magnet_origin_plate.point.x]
        Ysrc = [magnet_origin_plate.point.y]
        response = self.plate_to_camera(Xsrc,Ysrc)
        self.magnet_origin_camera.point.x = response.Xdst[0]
        self.magnet_origin_camera.point.y = response.Ydst[0]
        self.magnet_origin_undistorted = self.tf_listener.transformPoint(header.frame_id,self.magnet_origin_camera)
        # rospy.logwarn("x = %s\ty = %s" % (self.magnet_origin_undistorted.point.x,self.magnet_origin_undistorted.point.y))
        x = self.magnet_origin_undistorted.point.x
        y = self.magnet_origin_undistorted.point.y
        for contour in range(contour_count):
          # rospy.logwarn("x = %s\tx_list = %s" % (str(x),str(x_list[contour])))
          # rospy.logwarn("y = %s\ty_list = %s" % (str(y),str(y_list[contour])))
          dist_list.append(math.sqrt((x_list[contour] - x)**2 + (y_list[contour] - y)**2))
        # rospy.logwarn("dist_list = %s" % str(dist_list))
      except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        pass

      robot_list = []
      for contour in range(contour_count):
        x = x_list[contour]
        y = y_list[contour]
        theta = theta_list[contour]
        area = area_list[contour]
        ecc = ecc_list[contour]
        # Identify potential robots
        if ((self.robot_min_area < area) and (area < self.robot_max_area)) and ((self.robot_min_ecc < ecc) and (ecc < self.robot_max_ecc)):
          robot_list.append(contour)
        # rospy.logwarn("min_area = %s, max_area = %s, area = %s" % (str(self.robot_min_area),str(self.robot_max_area),str(area)))
        # rospy.logwarn("min_ecc = %s, max_ecc = %s, ecc = %s" % (str(self.robot_min_ecc),str(self.robot_max_ecc),str(ecc)))
        # rospy.logwarn("type(self.robot_min_area) = %s" % str(type(self.robot_min_area)))
          # self.robot_image_pose.header = header
          # self.robot_image_pose.pose.position.x = x_list[contour]
          # self.robot_image_pose.pose.position.y = y_list[contour]
          # self.robot_image_pose_pub.publish(self.robot_image_pose)
        # elif contour_count == 2:
        #   self.fly_image_pose.header = header
        #   self.fly_image_pose.pose.position.x = x
        #   self.fly_image_pose.pose.position.y = y
        #   self.fly_image_pose_pub.publish(self.fly_image_pose)

      # rospy.logwarn("robot_list = %s" % str(robot_list))
      # rospy.logwarn("dist_list = %s" % str(dist_list))
      if (0 < len(robot_list)) and (len(robot_list) <= len(dist_list)):
        robot_dist_list = [dist_list[robot] for robot in robot_list]
        # rospy.logwarn("robot_dist_list = %s" % str(robot_dist_list))
        robot = dist_list.index(min(robot_dist_list))
        # rospy.logwarn("robot = %s" % str(robot))

        self.robot_image_pose.header = header
        self.robot_image_pose.pose.position.x = x_list[robot]
        self.robot_image_pose.pose.position.y = y_list[robot]
        self.robot_image_pose_pub.publish(self.robot_image_pose)
        if 1 < contour_count:
          fly = min(list(set(range(contour_count)).difference([robot])))
          # rospy.logwarn("fly = %s" % str(fly))
          self.fly_image_pose.header = header
          self.fly_image_pose.pose.position.x = x_list[fly]
          self.fly_image_pose.pose.position.y = y_list[fly]
          self.fly_image_pose_pub.publish(self.fly_image_pose)

def main(args):
  rospy.init_node('ContourIdentifierTF', anonymous=True)
  ci = ContourIdentifier()
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
