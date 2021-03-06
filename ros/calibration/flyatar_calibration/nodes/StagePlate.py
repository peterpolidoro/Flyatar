#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('flyatar_calibration')
import rospy
import cv
import numpy
import tf
from cv_bridge import CvBridge, CvBridgeError
from pythonmodules import cvNumpy,CameraParameters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from plate_tf.srv import *
from track_image_contours.msg import ContourInfo

class Calibration():
  def __init__(self):
    self.initialized = False
    self.images_initialized = False
    self.pose_initialized = False
    self.arrays_initialized = False

    self.image_sub = rospy.Subscriber("camera/image_rect", Image, self.image_callback)
    self.contour_info_sub = rospy.Subscriber("ContourInfo",ContourInfo,self.contour_callback)
    # self.pose_sub = rospy.Subscriber("RobotImagePose", PoseStamped, self.pose_callback)

    cv.NamedWindow("Stage Plate Calibration", 1)

    self.robot_image_pose_camera = PoseStamped()
    self.robot_image_pose_rect = PoseStamped()
    self.robot_image_pose_plate = PoseStamped()

    self.bridge = CvBridge()
    self.color_max = 255
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    self.font_color = cv.CV_RGB(self.color_max,0,0)

    self.sample_dist_min = 20

    self.point_count_min = 10
    self.dist_min = 25

    self.error_text = ""
    self.robot_ecc = 0
    self.robot_area = 0
    self.robot_ecc_array = numpy.array([])
    self.robot_area_array = numpy.array([])
    self.robot_min_ecc = 1000000000
    self.robot_max_ecc = 0
    self.robot_min_area = 1000000000
    self.robot_max_area = 0

    # self.plate_point_array = numpy.zeros((1,3))
    # self.stage_point_array = numpy.zeros((1,3))
    (self.intrinsic_matrix,self.distortion_coeffs) = CameraParameters.intrinsic("rect")
    (self.rvec,self.tvec) = CameraParameters.extrinsic("plate")
    self.origin_points = cv.CreateMat(4,3,cv.CV_32FC1)
    self.origin_points_projected = cv.CreateMat(4,2,cv.CV_32FC1)
    self.checker_size = 15
    self.num_grid_lines = 9
    self.start_points_projected = cv.CreateMat(self.num_grid_lines,2,cv.CV_32FC1)
    self.end_points_projected = cv.CreateMat(self.num_grid_lines,2,cv.CV_32FC1)
    self.rotate_grid = False

    self.tf_listener = tf.TransformListener()
    got_trans = False
    while not got_trans:
      try:
        (self.camera_rect_trans,rot) = self.tf_listener.lookupTransform('/ImageRect', '/Camera', rospy.Time(0))
        got_trans = True
      except:
        rospy.logdebug("tf_listener.lookupTransform threw exception \n")

    rospy.wait_for_service('camera_to_plate')
    try:
      self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    self.initialized = True


  def initialize_images(self,cv_image):
    self.im_size = cv.GetSize(cv_image)
    (self.im_width,self.im_height) = cv.GetSize(cv_image)
    # self.im = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,1)
    self.im_display = cv.CreateImage(self.im_size,cv.IPL_DEPTH_8U,3)
    cv.Zero(self.im_display)
    self.images_initialized = True

  def draw_origin(self,im_color):
    cv.SetZero(self.origin_points)
    cv.SetReal2D(self.origin_points,1,0,self.checker_size) # x-direction
    cv.SetReal2D(self.origin_points,2,1,self.checker_size) # y-direction
    cv.SetReal2D(self.origin_points,3,2,self.checker_size) # z-direction
    axis_line_width = 3
    rect_line_width = 2
    cv.ProjectPoints2(self.origin_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
                      self.origin_points_projected)

    try:
      a = cvNumpy.mat_to_array(self.origin_points_projected)

      # origin point
      pt1 = tuple(a[0])

      # draw x-axis
      pt2 = tuple(a[1])
      cv.Line(im_color,pt1,pt2,cv.CV_RGB(self.color_max,0,0),axis_line_width)

      # draw y-axis
      pt2 = tuple(a[2])
      cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,self.color_max,0),axis_line_width)

      # draw z-axis
      pt2 = tuple(a[3])
      cv.Line(im_color,pt1,pt2,cv.CV_RGB(0,0,self.color_max),axis_line_width)

      # if self.image_plate_origin_found:
      #   self.image_plate_origin = pt1
      #   self.image_plate_origin_found = False

      # display_text = "image_plate_origin = " + str(self.image_plate_origin)
      # cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)
      # display_text = "image_plate_origin = (%0.0f, %0.0f)" % (self.image_plate_origin[0],self.image_plate_origin[1])
      # cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)
    except:
      pass

  def to_homo(self,array):
    array = numpy.append(array,numpy.ones((1,array.shape[1])),axis=0)
    return array

  def from_homo(self,array):
    return array[:-1,:]

  def draw_grid(self,im_color):
    points_range = (numpy.array(range(self.num_grid_lines)) - (self.num_grid_lines-1)/2) * self.checker_size
    points_zeros = numpy.zeros(points_range.shape)
    points_ones = numpy.ones(points_range.shape) * self.checker_size * (self.num_grid_lines - 1)/2

    x_start_points_array = numpy.array([-points_ones,points_range,points_zeros]).astype('float32')
    x_end_points_array = numpy.array([points_ones,points_range,points_zeros]).astype('float32')
    y_start_points_array = numpy.array([points_range,-points_ones,points_zeros]).astype('float32')
    y_end_points_array = numpy.array([points_range,points_ones,points_zeros]).astype('float32')

    if self.rotate_grid:
      x_start_points_array = self.from_homo(numpy.dot(self.T_stage_plate,self.to_homo(x_start_points_array)))
      x_end_points_array = self.from_homo(numpy.dot(self.T_stage_plate,self.to_homo(x_end_points_array)))
      y_start_points_array = self.from_homo(numpy.dot(self.T_stage_plate,self.to_homo(y_start_points_array)))
      y_end_points_array = self.from_homo(numpy.dot(self.T_stage_plate,self.to_homo(y_end_points_array)))
      self.rotate_grid = False

    x_start_points = cvNumpy.array_to_mat(x_start_points_array.transpose())
    x_end_points = cvNumpy.array_to_mat(x_end_points_array.transpose())
    y_start_points = cvNumpy.array_to_mat(y_start_points_array.transpose())
    y_end_points = cvNumpy.array_to_mat(y_end_points_array.transpose())

    axis_line_width = 1

    cv.ProjectPoints2(x_start_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
                      self.start_points_projected)
    cv.ProjectPoints2(x_end_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
                      self.end_points_projected)

    start_points = cvNumpy.mat_to_array(self.start_points_projected)
    end_points = cvNumpy.mat_to_array(self.end_points_projected)

    for line_n in range(self.num_grid_lines):
      cv.Line(im_color,tuple(start_points[line_n,:]),tuple(end_points[line_n,:]),cv.CV_RGB(self.color_max,0,0),axis_line_width)

    cv.ProjectPoints2(y_start_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
                      self.start_points_projected)
    cv.ProjectPoints2(y_end_points,
                      self.rvec,
                      self.tvec,
                      self.intrinsic_matrix,
                      self.distortion_coeffs,
                      self.end_points_projected)

    start_points = cvNumpy.mat_to_array(self.start_points_projected)
    end_points = cvNumpy.mat_to_array(self.end_points_projected)

    for line_n in range(self.num_grid_lines):
      cv.Line(im_color,tuple(start_points[line_n,:]),tuple(end_points[line_n,:]),cv.CV_RGB(0,self.color_max,0),axis_line_width)

  def image_callback(self,data):
    if self.initialized:
      try:
        cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
      except CvBridgeError, e:
        print e

      if not self.images_initialized:
        self.initialize_images(cv_image)

      cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

      if self.pose_initialized:
        try:
          (trans,rot_quat) = self.tf_listener.lookupTransform('Stage', 'Magnet', rospy.Time(0))
          magnet_stage_x = trans[0]
          magnet_stage_y = trans[1]
          magnet_stage_z = trans[2]

          if self.arrays_initialized:
            image_point_num = self.image_point_array.shape[1]
            for image_point_n in range(image_point_num):
              cv.Circle(self.im_display, (int(self.image_point_array[0,image_point_n]),int(self.image_point_array[1,image_point_n])), 5, cv.CV_RGB(self.color_max,0,self.color_max), cv.CV_FILLED)

          cv.Circle(self.im_display, (int(self.robot_image_pose_rect.pose.position.x),int(self.robot_image_pose_rect.pose.position.y)), 3, cv.CV_RGB(0,0,self.color_max), cv.CV_FILLED)
          # display_text = "robot_image_pose_camera.x = " + str(round(self.robot_image_pose_camera.pose.position.x,3))
          # cv.PutText(self.im_display,display_text,(25,25),self.font,self.font_color)
          # display_text = "robot_image_pose_camera.y = " + str(round(self.robot_image_pose_camera.pose.position.y,3))
          # cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)

          display_text = "RobotImage Pose with Respect to Plate = [%0.3f, %0.3f]" % (self.robot_image_pose_plate.pose.position.x,self.robot_image_pose_plate.pose.position.y)
          cv.PutText(self.im_display,display_text,(25,25),self.font,self.font_color)

          # display_text = "robot_image_pose_plate.x = " + str(round(self.robot_image_pose_plate.pose.position.x,3))
          # cv.PutText(self.im_display,display_text,(25,65),self.font,self.font_color)
          # display_text = "robot_image_pose_plate.y = " + str(round(self.robot_image_pose_plate.pose.position.y,3))
          # cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)

          display_text = "Magnet Pose with Respect to Stage = [%0.3f, %0.3f]" % (magnet_stage_x,magnet_stage_y)
          cv.PutText(self.im_display,display_text,(25,45),self.font,self.font_color)

          # display_text = "stage_state.x = " + str(round(self.stage_state.x,3))
          # cv.PutText(self.im_display,display_text,(25,105),self.font,self.font_color)
          # display_text = "stage_state.y = " + str(round(self.stage_state.y,3))
          # cv.PutText(self.im_display,display_text,(25,125),self.font,self.font_color)

          image_point_new = numpy.array([[self.robot_image_pose_rect.pose.position.x], [self.robot_image_pose_rect.pose.position.y]])
          plate_point_new = numpy.array([[self.robot_image_pose_plate.pose.position.x], [self.robot_image_pose_plate.pose.position.y],[0]])
          stage_point_new = numpy.array([[magnet_stage_x], [magnet_stage_y], [magnet_stage_z]])
          # rospy.logwarn("plate_point_new = \n%s", str(plate_point_new))
          # rospy.logwarn("stage_point_new = \n%s", str(stage_point_new))

          if self.arrays_initialized:
            plate_point_prev = self.plate_point_array[:,-1].reshape((3,1))
            # rospy.logwarn("plate_point_prev = \n%s", str(plate_point_prev))
            if self.dist_min < tf.transformations.vector_norm((plate_point_new - plate_point_prev)):
              self.image_point_array = numpy.append(self.image_point_array,image_point_new,axis=1)
              self.plate_point_array = numpy.append(self.plate_point_array,plate_point_new,axis=1)
              # rospy.logwarn("plate_point_array = \n%s", str(self.plate_point_array))
              self.stage_point_array = numpy.append(self.stage_point_array,stage_point_new,axis=1)
              # rospy.logwarn("stage_point_array = \n%s", str(self.stage_point_array))
              self.robot_ecc_array = numpy.append(self.robot_ecc_array,self.robot_ecc)
              self.robot_area_array = numpy.append(self.robot_area_array,self.robot_area)
          else:
            self.image_point_array = image_point_new
            self.plate_point_array = plate_point_new
            self.stage_point_array = stage_point_new
            self.arrays_initialized = True

          if self.point_count_min < self.plate_point_array.shape[1]:
            self.T_plate_stage = tf.transformations.superimposition_matrix(self.plate_point_array, self.stage_point_array)
            self.T_stage_plate = tf.transformations.inverse_matrix(self.T_plate_stage)
            # self.T_stage_plate = tf.transformations.superimposition_matrix(self.plate_point_array, self.stage_point_array)
            # self.T_plate_stage = tf.transformations.inverse_matrix(self.T_stage_plate)
            # tvector = tf.transformations.translation_from_matrix(self.T_plate_stage)
            tvector = tf.transformations.translation_from_matrix(self.T_stage_plate)
            display_text = "Translation Vector = [%0.3f, %0.3f, %0.3f]" % (tvector[0],tvector[1],tvector[2])
            cv.PutText(self.im_display,display_text,(25,65),self.font,self.font_color)
            # rospy.logwarn("tvec = \n%s",str(tvec))
            # quaternion = tf.transformations.quaternion_from_matrix(self.T_plate_stage)
            quaternion = tf.transformations.quaternion_from_matrix(self.T_stage_plate)
            display_text = "Quaternion = [%0.3f, %0.3f, %0.3f, %0.3f]" % (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            cv.PutText(self.im_display,display_text,(25,85),self.font,self.font_color)
            # rospy.logwarn("quaternion = \n%s",str(quaternion))
            # euler = tf.transformations.euler_from_matrix(self.T_plate_stage,'rxyz')
            # display_text = "Translation Vector = [%0.2f, %0.2f, %0.2f]" % (self.tvec[0],
            # cv.PutText(self.im_display,display_text,(25,65),self.font,self.font_color)
            # rospy.logwarn("euler = \n%s",str(euler))

            ecc_mean = numpy.mean(self.robot_ecc_array)
            ecc_std = numpy.std(self.robot_ecc_array)
            # rospy.logwarn("ecc_mean = %s, ecc_std = %s\n" % (ecc_mean, ecc_std))
            self.robot_min_ecc = ecc_mean - ecc_std*3
            if self.robot_min_ecc < 0:
              self.robot_min_ecc = 0
            self.robot_max_ecc = ecc_mean + ecc_std*3
            area_mean = numpy.mean(self.robot_area_array)
            area_std = numpy.std(self.robot_area_array)
            # rospy.logwarn("area_mean = %s, area_std = %s\n" % (area_mean, area_std))
            self.robot_min_area = area_mean - area_std*3
            if self.robot_min_area < 0:
              self.robot_min_area = 0
            self.robot_max_area = area_mean + area_std*3
            display_text = "robot_min_ecc = %0.3f, robot_max_ecc = %0.3f" % (self.robot_min_ecc, self.robot_max_ecc)
            cv.PutText(self.im_display,display_text,(25,105),self.font,self.font_color)
            display_text = "robot_min_area = %0.0f, robot_max_area = %0.0f" % (self.robot_min_area, self.robot_max_area)
            cv.PutText(self.im_display,display_text,(25,125),self.font,self.font_color)

            self.rotate_grid = True
            self.draw_grid(self.im_display)

          self.draw_origin(self.im_display)
          self.draw_grid(self.im_display)
        except (tf.LookupException, tf.ConnectivityException):
          pass


      cv.PutText(self.im_display,self.error_text,(25,185),self.font,self.font_color)

      cv.ShowImage("Stage Plate Calibration", self.im_display)
      cv.WaitKey(3)

  # def pose_callback(self,data):
  #   if not self.pose_initialized:
  #     self.pose_initialized = True
  #   self.robot_image_pose_camera.header = data.header
  #   self.robot_image_pose_camera.pose.position.x = data.pose.position.x
  #   self.robot_image_pose_camera.pose.position.y = data.pose.position.y
  #   self.robot_image_pose_rect = self.camera_to_rect_pose(self.robot_image_pose_camera)
  #   self.robot_image_pose_plate = self.camera_to_plate_pose(self.robot_image_pose_camera)

  def contour_callback(self,data):
    if self.initialized:
      header = data.header
      x_list = data.x
      y_list = data.y
      theta_list = data.theta
      area_list = data.area
      ecc_list = data.ecc
      contour_count = min(len(x_list),len(y_list),len(theta_list),len(area_list),len(ecc_list))

      if contour_count == 1:
        self.error_text = ""
        if not self.pose_initialized:
          self.pose_initialized = True
        self.robot_image_pose_camera.header = data.header
        self.robot_image_pose_camera.pose.position.x = x_list[0]
        self.robot_image_pose_camera.pose.position.y = y_list[0]
        self.robot_image_pose_rect = self.camera_to_rect_pose(self.robot_image_pose_camera)
        self.robot_image_pose_plate = self.camera_to_plate_pose(self.robot_image_pose_camera)
        self.robot_area = area_list[0]
        self.robot_ecc = ecc_list[0]
      else:
        rospy.logwarn("Error! More than one object detected!")
        self.error_text = "Error! More than one object detected!"

      # for contour in range(contour_count):
      #   x = x_list[contour]
      #   y = y_list[contour]
      #   theta = theta_list[contour]
      #   area = area_list[contour]
      #   ecc = ecc_list[contour]
      #   # Identify robot
      #   if ((self.robot_min_area < area) and (area < self.robot_max_area)) and ((self.robot_min_ecc < ecc) and (ecc < self.robot_max_ecc)):
      #     self.robot_image_pose.header = header
      #     self.robot_image_pose.pose.position.x = x
      #     self.robot_image_pose.pose.position.y = y
      #     self.robot_image_pose_pub.publish(self.robot_image_pose)
      #   elif contour_count == 2:
      #     self.fly_image_pose.header = header
      #     self.fly_image_pose.pose.position.x = x
      #     self.fly_image_pose.pose.position.y = y
      #     self.fly_image_pose_pub.publish(self.fly_image_pose)

  def camera_to_rect_pose(self,input_pose):
    output_pose = PoseStamped()
    output_pose.pose.position.x = self.camera_rect_trans[0] + input_pose.pose.position.x
    # rospy.logwarn("input_pose.pose.position.x \n%s",str(input_pose.pose.position.x))
    # rospy.logwarn("self.camera_rect_trans[0] \n%s",str(self.camera_rect_trans[0]))
    # rospy.logwarn("self.output_pose.pose.position.x \n%s",str(output_pose.pose.position.x))
    output_pose.pose.position.y = self.camera_rect_trans[1] + input_pose.pose.position.y
    # rospy.logwarn("input_pose.pose.position.y \n%s",str(input_pose.pose.position.y))
    # rospy.logwarn("self.camera_rect_trans[1] \n%s",str(self.camera_rect_trans[1]))
    # rospy.logwarn("self.output_pose.pose.position.y \n%s",str(output_pose.pose.position.y))
    return output_pose

  def camera_to_plate_pose(self,input_pose):
    output_pose = PoseStamped()
    Xsrc = [input_pose.pose.position.x]
    Ysrc = [input_pose.pose.position.y]
    # rospy.logwarn("Xsrc \n%s",str(Xsrc))
    # rospy.logwarn("Ysrc \n%s",str(Ysrc))
    response = self.camera_to_plate(Xsrc,Ysrc)
    # rospy.logwarn("Xdst \n%s",str(response.Xdst))
    # rospy.logwarn("Ydst \n%s",str(response.Ydst))
    output_pose.pose.position.x = response.Xdst[0]
    output_pose.pose.position.y = response.Ydst[0]
    return output_pose

if __name__ == '__main__':
  rospy.init_node('StagePlate')
  cal = Calibration()

  while not rospy.is_shutdown():
    rospy.spin()

  cv.DestroyAllWindows()
