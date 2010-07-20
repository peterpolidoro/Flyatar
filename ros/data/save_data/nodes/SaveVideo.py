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
from save_data.msg import BagInfo, VideoInfo

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class SaveVideo:
    def __init__(self):
        self.initialized = False
        self.working_dir_base = os.path.expanduser("~/Videos")
        chdir(self.working_dir_base)
        self.working_dir_base = self.working_dir_base + "/" + time.strftime("%Y-%m-%d")
        chdir(self.working_dir_base)

        self.working_dir = None

        self.bag_info_sub = rospy.Subscriber("bag_info",BagInfo,self.bag_info_callback)
        self.bag_info = BagInfo()
        self.bag_info.bag_name = "None"

        self.saving_images = False
        self.saving_video = False
        self.ready_to_save_video = False

        self.video_info_pub = rospy.Publisher("video_info",VideoInfo)
        self.video_info = VideoInfo()
        self.video_info.ready_to_record = False

        self.image_frame = rospy.get_param("save_image_frame")
        self.image_sub = rospy.Subscriber(self.image_frame, Image, self.image_callback)

        self.bridge = CvBridge()
        self.image_number = 0
        self.frame_rate = rospy.get_param("framerate",30)
        self.video_format = rospy.get_param("save_video_format","flv")

        # self.saving_images_started = False
        # self.last_image_time = None
        # self.rate = rospy.Rate(10)     # Hz
        # self.time_limit = 3

        self.initialized = True

        # self.color_max = 255
        # self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        # self.video_initialized = False

    def bag_info_callback(self,data):
        if self.initialized:
            self.bag_info = data
            bag_name = data.bag_name
            ready_to_play = data.ready_to_play
            finished_playing = data.finished_playing
            end_of_bag_files = data.end_of_bag_files

            if end_of_bag_files:
                rospy.logwarn("End of bag files.")
                return
            elif self.saving_video:
                self.video_info.ready_to_record = False
            elif (bag_name != "") and ready_to_play and (not finished_playing) and (not self.saving_images):
                self.working_dir = self.working_dir_base + "/" + bag_name
                chdir(self.working_dir)
                self.saving_images = True
                self.video_info.ready_to_record = True
                rospy.logwarn("Saving images.")
            elif finished_playing:
                self.saving_images = False
                self.ready_to_save_video = True
                self.video_info.ready_to_record = False
                rospy.logwarn("Saving video.")
            self.video_info_pub.publish(self.video_info)

    def save_png(self,cv_image):
        image_name = "{num:06d}.png".format(num=self.image_number)
        cv.SaveImage(image_name,cv_image)

    def image_callback(self,data):
        if (self.working_dir is not None) and (not self.saving_video):
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

            # rospy.loginfo("Saved image {num:06d}\n".format(num=self.image_number))
            self.image_number += 1
            # if not self.images_initialized:
            #     self.initialize_images(cv_image)

            # cv.CvtColor(cv_image,self.im_display,cv.CV_GRAY2RGB)

    def save_video(self):
        self.saving_video = True
        chdir(self.working_dir_base)
        bag_name = self.bag_info.bag_name
        subprocess.check_call('ffmpeg -f image2 -i ' + \
                               bag_name + '/%06d.png ' + \
                               '-r ' + str(self.frame_rate) + ' ' + \
                               '-s 640x480 -mbd rd -trellis 2 -cmp 2 -subcmp 2 -pass 1/2 ' + \
                               bag_name + '.mpg',shell=True)
        rospy.logwarn('Saved video %s' % + (bag_name + '.mpg'))
        self.saving_video = False
        self.ready_to_save_video = False

    def main(self):
        while not rospy.is_shutdown():
            if self.ready_to_save_video:
                self.save_video()
            # if self.saving_images_started and (self.last_image_time is not None):
            #     t = rospy.get_time()
            #     dt = t - self.last_image_time
            #     # rospy.logwarn("dt = %s" % (str(dt)))
            #     if self.time_limit < dt:
            #         self.save_video()
            # self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('SaveVideo',log_level=rospy.INFO)
    sv = SaveVideo()
    sv.main()
