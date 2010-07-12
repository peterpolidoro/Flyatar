#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time,os,subprocess

class SaveImages:

    def __init__(self):
        os.chdir(os.path.expanduser("~/Videos"))
        self.working_dir = time.strftime("%Y-%m-%d-%H-%M-%S")
        os.mkdir(self.working_dir)
        os.chdir(self.working_dir)

        self.image_frame = rospy.get_param("save_image_frame")
        self.image_sub = rospy.Subscriber(self.image_frame, Image, self.image_callback)

        self.bridge = CvBridge()
        self.image_number = 0
        self.frame_rate = rospy.get_param("framerate","30")
        self.video_format = rospy.get_param("save_video_format","flv")

        # self.color_max = 255
        # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        # self.video_initialized = False

    def save_png(self,cv_image):
        image_name = "{num:06d}.png".format(num=self.image_number)
        cv.SaveImage(image_name,cv_image)

    # def write_frame(self,cv_image):
    #     cv.WriteFrame(self.video_writer,cv_image)

    # def initialize_video(self,cv_image):
    #     self.video_writer = cv.CreateVideoWriter(self.working_dir+".mjpg",
    #                                              cv.CV_FOURCC('M','J','P','G'),
    #                                              self.frame_rate,
    #                                              cv.GetSize(cv_image))

        # self.im_display = cv.CreateImage(cv.GetSize(cv_image),cv.IPL_DEPTH_8U,3)
        # self.video_initialized = True

    def image_callback(self,data):
        # Convert ROS image to OpenCV image
        try:
          cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "passthrough"))
        except CvBridgeError, e:
          print e

        self.save_png(cv_image)

        # if self.video_format in "png":
        #     self.save_png(cv_image)
        # else:
        #     if not self.video_initialized:
        #         self.initialize_video(cv_image)
        #     self.write_frame(cv_image)

        rospy.loginfo("Saved image {num:06d}\n".format(num=self.image_number))
        self.image_number += 1
        # if not self.images_initialized:
        #     self.initialize_images(cv_image)

        # cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

if __name__ == '__main__':
    rospy.init_node('SaveImages',log_level=rospy.INFO)
    si = SaveImages()

    while not rospy.is_shutdown():
        rospy.spin()

    os.chdir(os.path.expanduser("~/Videos"))
    if si.video_format in "flv":
        subprocess.check_call(['ffmpeg','-f','image2',
                               '-i',si.working_dir+'/%06d.png',
                               '-sameq',
                               '-ar','44100',
                               '-ab','64k',
                               '-ac','2',
                               '-r',str(si.frame_rate),
                               '-s','640x480',
                               si.working_dir+'.flv'])
        print 'Saved video ' + si.working_dir + '.flv'
    elif si.video_format in "gif":
        subprocess.check_call(['ffmpeg','-f','image2',
                               '-i',si.working_dir+'/%06d.png',
                               '-sameq',
                               '-r',str(si.frame_rate),
                               '-s','640x480',
                               '-pix_fmt','rgb24',
                               si.working_dir+'.gif'])
        print 'Saved video ' + si.working_dir + '.gif'
    elif si.video_format in "avi":
        subprocess.check_call(['ffmpeg','-f','image2',
                               '-i',si.working_dir+'/%06d.png',
                               '-sameq',
                               '-r',str(si.frame_rate),
                               '-s','640x480',
                               si.working_dir+'.avi'])
        print 'Saved video ' + si.working_dir + '.avi'
