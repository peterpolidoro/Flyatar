#!/usr/bin/env python
from __future__ import division
import roslib
roslib.load_manifest('save_data')
import sys
import rospy
import time,os,subprocess
from save_data.msg import BagInfo

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

        self.NULL = open('/dev/null', 'w')

        self.bag_set = self.find_bag_set()

        self.initilized = True

    def find_bag_set(self):
        p_ls_bag = subprocess.Popen('ls *.bag',shell=True,stdout=subprocess.PIPE,stderr=self.NULL)
        out = p_ls_bag.stdout.readlines()
        bag_set = set([s.rstrip() for s in out])
        return bag_set

    def play_bag_file(self,bag_file):
        rospy.loginfo("Playing %s" % (bag_file))
        subprocess.check_call('rosbag play ' + bag_file,shell=True)

    def main(self):
        if 0 < len(self.bag_set):
            for bag_file in self.bag_set:
                if (not rospy.is_shutdown()):
                    self.bag_info.bag_name = bag_file
                    self.bag_info_pub.publish(self.bag_info)
                    time.sleep(1)
                    self.play_bag_file(bag_file)
        else:
            rospy.logwarn("No bag files in %s" % (self.working_dir))

if __name__ == '__main__':
    rospy.init_node('PlayBags',log_level=rospy.INFO)
    pb = PlayBags()
    pb.main()
