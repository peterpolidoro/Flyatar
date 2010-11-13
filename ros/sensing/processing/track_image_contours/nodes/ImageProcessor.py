#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('track_image_contours')
import sys
import rospy
import cv
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from track_image_contours.msg import ContourInfo
from cv_bridge import CvBridge, CvBridgeError
import math
import time

class ImageProcessor:

  def __init__(self):

    # Image Initialization
    self.images_initialized = False
    self.initialized = False

    # Listener/Subscribers
    self.image_sub = rospy.Subscriber("camera/image_rect",Image,self.image_callback)
    self.tf_listener = tf.TransformListener()

    # Broadcaster/Publishers
    self.image_processed_pub = rospy.Publisher("camera/image_processed",Image)
    self.image_diff_pub = rospy.Publisher("camera/image_diff",Image)
    self.image_foreground_pub = rospy.Publisher("camera/image_foreground",Image)
    self.contour_info_pub = rospy.Publisher("ContourInfo",ContourInfo)

    # Contour Info
    self.contour_info = ContourInfo()
    self.x0_list = []
    self.y0_list = []
    self.theta_list = []
    self.area_list = []
    self.ecc_list = []
    self.contour_count = 0
    self.contour_count_max = int(rospy.get_param("contour_count_max","2"))
    self.min_ecc = 1.75
    self.image_sum_min = 100

    # Robot Info
    self.robot_visible = bool(rospy.get_param("robot_visible","true"))
    if (not self.robot_visible) and (1 < self.contour_count_max):
      self.contour_count_max -= 1

    # Image Windows
    self.display_images = bool(rospy.get_param("image_processor_display_images","true"))
    # self.display_images = True
    if self.display_images:
      cv.NamedWindow("Processed Image", 1)
      cv.NamedWindow("Diff Image", 1)
      cv.NamedWindow("Foreground Image", 1)
      cv.NamedWindow("Background Image", 1)

    # OpenCV
    self.max_8U = 255
    self.color_max = 255
    self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
    self.storage = cv.CreateMemStorage()
    self.bridge = CvBridge()

    # Coordinate Systems
    self.CS_initialized = False
    max_tries = 100
    tries = 0
    while (not self.CS_initialized) and (tries < max_tries):
      try:
        self.output_coordinates = rospy.get_param("ImageProcessor_OutputCoordinates","Camera")
        self.ROIPlateImage_origin = PointStamped()
        self.ROIPlateImage_origin.header.frame_id = "ROIPlateImage"
        self.ROIPlateImage_origin.point.x = 0
        self.ROIPlateImage_origin.point.y = 0
        self.undistorted_ROIPlateImage_origin = self.tf_listener.transformPoint("ImageRect",self.ROIPlateImage_origin)
        self.PlateImage_origin = PointStamped()
        self.PlateImage_origin.header.frame_id = "PlateImage"
        self.PlateImage_origin.point.x = 0
        self.PlateImage_origin.point.y = 0
        self.ROIPlateImage_PlateImage_origin = self.tf_listener.transformPoint("ROIPlateImage",self.PlateImage_origin)
        self.ROIPlateImage_point = PointStamped()
        self.ROIPlateImage_point.header.frame_id = "ROIPlateImage"
        time.sleep(0.1)
        self.CS_initialized = True
      except (tf.LookupException, tf.ConnectivityException):
        pass
      tries += 1

    # Mask Info
    self.mask_radius = int(rospy.get_param("mask_radius","225"))

    self.initialized = True

  def initialize_images(self,cv_image):
    self.undistorted_size = cv.GetSize(cv_image)
    (self.undistorted_width,self.undistorted_height) = cv.GetSize(cv_image)

    # ROI Setup
    self.ROIPlateImage_width = int(rospy.get_param("ROIPlateImage_width","480"))
    self.ROIPlateImage_height = int(rospy.get_param("ROIPlateImage_height","480"))
    self.ROIPlateImage_size = (self.ROIPlateImage_width,self.ROIPlateImage_height)
    self.ROIPlateImage_cvrect = (int(self.undistorted_ROIPlateImage_origin.point.x),
                                 int(self.undistorted_ROIPlateImage_origin.point.y),
                                 int(self.ROIPlateImage_width),
                                 int(self.ROIPlateImage_height))


    self.im = cv.CreateImage(self.undistorted_size,cv.IPL_DEPTH_8U,1)
    self.im_processed = cv.CreateImage(self.undistorted_size,cv.IPL_DEPTH_8U,3)
    cv.SetImageROI(self.im_processed,self.ROIPlateImage_cvrect)
    self.im_processed2 = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,3)
    self.im_mask = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_background = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_foreground = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_foreground_binary = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_dilate = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_erode = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_zeros = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_contour = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,1)
    self.im_contour_display = cv.CreateImage(self.ROIPlateImage_size,cv.IPL_DEPTH_8U,3)
    cv.Zero(self.im_zeros)
    # self.center_ROIx = self.ROIwidth//2
    # self.center_ROIy = self.ROIheight//2
    cv.Zero(self.im_mask)
    cv.Circle(self.im_mask,
              (int(self.ROIPlateImage_PlateImage_origin.point.x),int(self.ROIPlateImage_PlateImage_origin.point.y)),
              int(self.mask_radius), self.color_max, cv.CV_FILLED)
    # Create Background image
    # First image is background unless one can be loaded
    cv.SetImageROI(cv_image,self.ROIPlateImage_cvrect)
    try:
      self.im_background = cv.LoadImage("/cameras/background.png",cv.CV_LOAD_IMAGE_GRAYSCALE)
    except:
      cv.And(cv_image,self.im_mask,self.im_background)
      cv.SaveImage("/cameras/background.png",self.im_background)
    self.images_initialized = True

  def find_slope_ecc(self,A,B,C,D):
    slope = float('NaN')
    ecc = 0
    if C == 0:
      # divide by zero
      return slope,ecc
    inside = A*A + 4*B*C - 2*A*D + D*D
    if inside < 0:
      # complex answer
      return slope,ecc
    inside = math.sqrt(inside)
    evalA = 0.5*(A+D-inside)
    evalB = 0.5*(A+D+inside)
    evecA1 = (-A+D+inside)/(-2*C)
    evecB1 = (-A+D-inside)/(-2*C)
    rise = 1
    try:
      if evalB < evalA:
        run = evecA1
        ecc = evalA/evalB
      else:
        run = evecB1
        ecc = evalB/evalA
      slope = rise/run
    except:
      pass
    return slope,ecc

  def moments_to_parameters(self,moments):
    Mu00 = cv.GetSpatialMoment(moments,0,0)
    Mu10 = cv.GetSpatialMoment(moments,1,0)
    Mu01 = cv.GetSpatialMoment(moments,0,1)
    if Mu00 != 0:
      x0 = Mu10/Mu00
      y0 = Mu01/Mu00
    else:
      x0 = 0
      y0 = 0
    Uu11 = cv.GetCentralMoment(moments,1,1)
    Uu20 = cv.GetCentralMoment(moments,2,0)
    Uu02 = cv.GetCentralMoment(moments,0,2)
    area = Mu00
    slope,ecc = self.find_slope_ecc(Uu20,Uu11,Uu11,Uu02)
    return x0,y0,area,slope,ecc

  def draw_slope_line(self,im,x0,y0,slope,ecc):
    if (not math.isnan(slope)) and (self.min_ecc < ecc):
      # only draw if sufficient orientation info
      height,width = cv.GetSize(im)
      y0 = height-y0

      # line segment for orientation
      xmin = 0
      ymin = 0
      xmax = width-1
      ymax = height-1

      # ax+by+c=0
      a=-slope
      b=-1
      c=y0-a*x0

      x1=xmin
      y1=-(c+a*x1)/b
      if y1 < ymin:
        y1 = ymin
        x1 = -(c+b*y1)/a
      elif y1 > ymax:
        y1 = ymax
        x1 = -(c+b*y1)/a

      x2=xmax
      y2=-(c+a*x2)/b
      if y2 < ymin:
        y2 = ymin
        x2 = -(c+b*y2)/a
      elif y2 > ymax:
        y2 = ymax
        x2 = -(c+b*y2)/a

      if 1:
        xg = width
        xo = 0
      else:
        xg = -width
        xo = width-1
      yg = height
      yo = 0

      x1 = x1/width*xg+xo
      x2 = x2/width*xg+xo

      y1 = (height-y1)/height*yg+yo
      y2 = (height-y2)/height*yg+yo
      cv.Line(im,(int(x1),int(y1)),(int(x2),int(y2)),cv.CV_RGB(0,self.color_max,0))

  def process_contour(self,contour):
    im_contour_mask = cv.CloneImage(self.im_zeros)
    cv.DrawContours(im_contour_mask,self.contour_seq,cv.CV_RGB(0,0,self.color_max),cv.CV_RGB(0,self.color_max,0),-1,cv.CV_FILLED)
    cv.And(self.im_foreground,im_contour_mask,self.im_contour)
    moments = cv.Moments(self.im_contour)
    (ROIPlateImage_x0,ROIPlateImage_y0,area,slope,ecc) = self.moments_to_parameters(moments)
    if not math.isnan(slope):
      if self.min_ecc < ecc:
        theta = math.atan2(-slope,1)
      else:
        theta = 0
    else:
      theta = 0

    # Convert from ROIPlateImage coordinates to Camera coordinates
    self.ROIPlateImage_point.point.x = ROIPlateImage_x0
    self.ROIPlateImage_point.point.y = ROIPlateImage_y0
    self.Output_point = self.tf_listener.transformPoint(self.output_coordinates,self.ROIPlateImage_point)

    # Save contour info
    self.x0_list.append(self.Output_point.point.x)
    self.y0_list.append(self.Output_point.point.y)
    self.theta_list.append(theta)
    self.area_list.append(area)
    self.ecc_list.append(ecc)

    if self.display_images:
      cv.NamedWindow("Contour %s"%str(self.contour_count), 1)
      cv.CvtColor(self.im_contour,self.im_contour_display,cv.CV_GRAY2RGB)
      display_text = "Area = " + str(int(area))
      cv.PutText(self.im_contour_display,display_text,(25,25),self.font,cv.CV_RGB(self.color_max,0,0))
      display_text = "ecc = " + str(round(ecc,2))
      cv.PutText(self.im_contour_display,display_text,(int(self.ROIPlateImage_width/2),25),self.font,cv.CV_RGB(self.color_max,0,0))
      cv.Circle(self.im_contour_display,(int(ROIPlateImage_x0),int(ROIPlateImage_y0)),4,cv.CV_RGB(0,self.color_max,0))
      self.draw_slope_line(self.im_contour_display,ROIPlateImage_x0,ROIPlateImage_y0,slope,ecc)
      display_text = "x0 = " + str(int(self.Output_point.point.x))
      cv.PutText(self.im_contour_display,display_text,(25,45),self.font,cv.CV_RGB(self.color_max,0,0))
      display_text = "y0 = " + str(int(self.Output_point.point.y))
      cv.PutText(self.im_contour_display,display_text,(int(self.ROIPlateImage_width/2),45),self.font,cv.CV_RGB(self.color_max,0,0))
      display_text = "theta = " + str(round(theta,2))
      cv.PutText(self.im_contour_display,display_text,(25,65),self.font,cv.CV_RGB(self.color_max,0,0))
      display_text = "output coordinates = " + self.output_coordinates
      cv.PutText(self.im_contour_display,display_text,(25,85),self.font,cv.CV_RGB(self.color_max,0,0))
      cv.ShowImage("Contour %s"%str(self.contour_count),self.im_contour_display)

  def image_callback(self,data):
    if not self.initialized:
      return

    # Convert ROS image to OpenCV image
    try:
      cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
    except CvBridgeError, e:
      print e

    if not self.images_initialized:
      if self.CS_initialized:
        self.initialize_images(cv_image)
      else:
        return

    self.contour_info.header.stamp = rospy.Time.now()
    self.contour_info.header.frame_id = self.output_coordinates
    self.x0_list = []
    self.y0_list = []
    self.theta_list = []
    self.area_list = []
    self.ecc_list = []

    self.im = cv_image
    cv.SetImageROI(self.im,self.ROIPlateImage_cvrect)

    # Look for new diff_threshold value
    self.diff_threshold = int(rospy.get_param("diff_threshold","30"))

    # Apply mask
    cv.And(self.im,self.im_mask,self.im)

    # Subtract background
    cv.AbsDiff(self.im,self.im_background,self.im_foreground)
    if self.display_images:
      cv.ShowImage("Diff Image", self.im_foreground)

    # Publish diff image
    try:
      self.image_diff_pub.publish(self.bridge.cv_to_imgmsg(self.im_foreground,"passthrough"))
    except CvBridgeError, e:
      print e

    # Threshold
    cv.Threshold(self.im_foreground,self.im_foreground_binary,self.diff_threshold,self.max_8U,cv.CV_THRESH_BINARY)
    cv.Threshold(self.im_foreground,self.im_foreground,self.diff_threshold,self.max_8U,cv.CV_THRESH_TOZERO)
    # cv.Dilate(self.im_foreground,self.im_dilate)
    # cv.Erode(self.im_dilate,self.im_erode)
    # cv.ShowImage("Dilate Plus Erode Image", self.im_erode)

    # Find contours
    image_sum = cv.Sum(self.im_foreground_binary)
    if self.image_sum_min < image_sum[0]:
      self.contour_seq = cv.FindContours(self.im_foreground_binary,self.storage,mode=cv.CV_RETR_CCOMP)
    else:
      self.contour_seq = None

    # Convert to color for display image
    cv.CvtColor(self.im,self.im_processed,cv.CV_GRAY2RGB)

    # Draw contours
    if self.contour_seq:
      cv.DrawContours(self.im_processed,self.contour_seq,cv.CV_RGB(0,0,self.color_max),cv.CV_RGB(0,self.color_max,0),1,1)

    # Process contours
    self.contour_count = 0
    while(self.contour_seq and (self.contour_count < self.contour_count_max)):
      self.contour_count += 1
      self.process_contour(self.contour_seq)
      self.contour_seq = self.contour_seq.h_next()

    # display_text = str(self.contour_count)
    # cv.PutText(self.im_display,display_text,(25,25),self.font,cv.CV_RGB(self.color_max,0,0))

    if self.display_images:
      cv.ShowImage("Processed Image", self.im_processed)
      cv.ShowImage("Foreground Image", self.im_foreground)
      cv.ShowImage("Background Image", self.im_background)
      cv.WaitKey(3)

    # Publish foreground image
    try:
      self.image_foreground_pub.publish(self.bridge.cv_to_imgmsg(self.im_foreground,"passthrough"))
    except CvBridgeError, e:
      print e

    # Publish contour info
    if self.contour_count != 0:
      self.contour_info.x = self.x0_list
      self.contour_info.y = self.y0_list
      self.contour_info.theta = self.theta_list
      self.contour_info.area = self.area_list
      self.contour_info.ecc = self.ecc_list
      self.contour_info_pub.publish(self.contour_info)

    # Publish processed image
    try:
      cv.Copy(self.im_processed,self.im_processed2)
      self.image_processed_pub.publish(self.bridge.cv_to_imgmsg(self.im_processed2,"passthrough"))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('ImageProcessor', anonymous=True)
  ip = ImageProcessor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
