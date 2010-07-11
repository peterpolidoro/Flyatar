#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf
# import numpy, math
# from geometry_msgs.msg import PoseStamped
# from plate_tf.srv import *
# import kalman_filter as kf
# import stop_walk as sw
# import choose_orientation as co

class StagePlateTFBroadcaster:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)

        self.stage_plate_offset_x = rospy.get_param("stage_plate_offset_x")
        self.stage_plate_offset_y = rospy.get_param("stage_plate_offset_y")
        self.stage_plate_quat_z = rospy.get_param("stage_plate_quat_z")
        self.stage_plate_quat_w = rospy.get_param("stage_plate_quat_w")

        self.initialized = True

    def broadcast(self):
        while not rospy.is_shutdown():
            if self.initialized:
                # try:
                self.tf_broadcaster.sendTransform((self.stage_plate_offset_x, self.stage_plate_offset_y, 0),
                                                  (0, 0, self.stage_plate_quat_z, self.stage_plate_quat_w),
                                                  rospy.Time.now(),
                                                  "Stage",
                                                  "Plate")
                # except (tf.LookupException, tf.ConnectivityException, rospy.service.ServiceException):
                #     pass

                self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('stage_plate_tf_broadcaster')
    sptb = StagePlateTFBroadcaster()
    sptb.broadcast()
