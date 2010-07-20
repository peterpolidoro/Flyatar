#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import time,os,subprocess
from save_data.msg import BagInfo, VideoInfo

def chdir(dir):
    try:
        os.chdir(dir)
    except (OSError):
        os.mkdir(dir)
        os.chdir(dir)

class PlayBags:
    def __init__(self):
        self.initialized = False
        self.working_dir = os.path.expanduser("~/Bags")
        chdir(self.working_dir)
        self.working_dir = self.working_dir + "/Play"
        chdir(self.working_dir)

        self.bag_info_pub = rospy.Publisher("bag_info",BagInfo)
        self.bag_info = BagInfo()

        self.video_info_sub = rospy.Subscriber("video_info",VideoInfo,self.video_info_callback)
        self.video_info = VideoInfo()
        self.video_info.ready_to_record = False

        self.NULL = open('/dev/null', 'w')

        self.bag_set = self.find_bag_set()

        self.initilized = True

    def video_info_callback(self,data):
        if self.initialized:
            self.video_info = data

    def find_bag_set(self):
        p_ls_bag = subprocess.Popen('ls *.bag',shell=True,stdout=subprocess.PIPE,stderr=self.NULL)
        out = p_ls_bag.stdout.readlines()
        bag_set = set([s.rstrip() for s in out])
        return bag_set

    def play_bag_file(self,bag_file):
        # rospy.loginfo("Playing %s" % (bag_file))
        subprocess.check_call('rosbag play ' + bag_file,shell=True)

    def main(self):
        if 0 < len(self.bag_set):
            self.bag_info.end_of_bag_files = False
            for bag_file in self.bag_set:
                bag_name,bag_ext = os.path.splitext(bag_file)
                self.bag_info.bag_name = bag_name
                self.bag_info.ready_to_play = True
                self.bag_info.finished_playing = False
                while (not rospy.is_shutdown()) and (not self.video_info.ready_to_record):
                    self.bag_info_pub.publish(self.bag_info)
                    rospy.logwarn("stuck in loop....")
                    time.sleep(0.1)

                while (not rospy.is_shutdown()) and (self.video_info.ready_to_record):
                    self.video_info.ready_to_record = False
                    rospy.logwarn("Playing bag file...")
                    self.play_bag_file(bag_file)
                    self.bag_info.finished_playing = True
                    self.bag_info_pub.publish(self.bag_info)
        else:
            rospy.logwarn("No bag files in %s" % (self.working_dir))

        self.bag_info.end_of_bag_files = True
        self.bag_info_pub.publish(self.bag_info)

if __name__ == '__main__':
    rospy.init_node('PlayBags',log_level=rospy.INFO)
    pb = PlayBags()
    pb.main()
