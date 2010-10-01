#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('track_image_contours')
import sys
import rospy
from track_image_contours.msg import ContourInfo
from plate_tf.msg import ImagePose
from geometry_msgs.msg import PointStamped,PoseArray,Pose
import plate_tf.srv
import tf,math
import copy

class ContourIdentifier:

  def __init__(self):
    self.initialized = False

    # Listener/Subscribers
    self.contour_info_sub = rospy.Subscriber("ContourInfo",ContourInfo,self.contour_callback)
    self.tf_listener = tf.TransformListener()

    # Broadcaster/Publishers
    self.image_pose_pub = rospy.Publisher("ImagePose/Robot",PoseStamped)

    # Pose
    self.image_pose = ImagePose()
    self.robot_image_pose = Pose()
    self.fly_image_pose = Pose()
    self.fly_image_pose_array = PoseArray()

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

    rospy.wait_for_service('plate_to_camera')
    try:
      self.plate_to_camera = rospy.ServiceProxy('plate_to_camera', plate_tf.srv.PlateCameraConversion)
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
        for contour_index in range(contour_count):
          dist_list.append(math.sqrt((x_list[contour_index] - x)**2 + (y_list[contour_index] - y)**2))
        # rospy.logwarn("dist_list = %s" % str(dist_list))
      except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        pass

      robot_index_list = []
      for contour_index in range(contour_count):
        x = x_list[contour_index]
        y = y_list[contour_index]
        theta = theta_list[contour_index]
        area = area_list[contour_index]
        ecc = ecc_list[contour_index]

        # Identify potential robots
        if ((self.robot_min_area < area) and (area < self.robot_max_area)) and ((self.robot_min_ecc < ecc) and (ecc < self.robot_max_ecc)):
          robot_index_list.append(contour_index)

      # Check to make sure there are potential robots and that dist_list has been filled previously
      if (0 < len(robot_index_list)) and (len(robot_index_list) <= len(dist_list)):
        robot_dist_list = [dist_list[robot_index] for robot_index in robot_index_list]
        # rospy.logwarn("robot_dist_list = %s" % str(robot_dist_list))
        robot_index = dist_list.index(min(robot_dist_list))
        # rospy.logwarn("robot = %s" % str(robot))

        self.image_pose.header = header
        self.robot_image_pose.position.x = x_list[robot_index]
        self.robot_image_pose.position.y = y_list[robot_index]
        q = tf.transformations.quaternion_about_axis(theta_list[robot_index], (0, 0, 1))
        self.robot_image_pose.orientation.x = q[0]
        self.robot_image_pose.orientation.y = q[1]
        self.robot_image_pose.orientation.z = q[2]
        self.robot_image_pose.orientation.w = q[3]
        self.image_pose.robot_image_pose = self.robot_image_pose
        if 1 < contour_count:
          fly_index_list = list(set(range(contour_count)).difference([robot_index]))
          self.fly_image_pose_array.poses = []
          for fly_index in fly_index_list:
            self.fly_image_pose.position.x = x_list[fly_index]
            self.fly_image_pose.position.y = y_list[fly_index]
            q = tf.transformations.quaternion_about_axis(theta_list[fly_index], (0, 0, 1))
            self.fly_image_pose.orientation.x = q[0]
            self.fly_image_pose.orientation.y = q[1]
            self.fly_image_pose.orientation.z = q[2]
            self.fly_image_pose.orientation.w = q[3]
            self.fly_image_pose_array.poses.append(copy.deepcopy(self.fly_image_pose))
        else:
          self.fly_image_pose_array = PoseArray()

        self.image_pose.fly_image_poses = self.fly_image_pose_array.poses
        self.image_pose_pub.publish(self.image_pose)

def main(args):
  rospy.init_node('ContourIdentifierTF', anonymous=True)
  ci = ContourIdentifier()
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
