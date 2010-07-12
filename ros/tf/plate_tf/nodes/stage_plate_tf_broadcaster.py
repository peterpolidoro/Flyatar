#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf
# import numpy, math
from geometry_msgs.msg import PointStamped
# from plate_tf.srv import *
import kalman_filter as kf
import copy
from plate_tf.msg import StopState

class StagePlateTFBroadcaster:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.stop_state_sub = rospy.Subscriber('StopState',StopState,self.stop_state_callback)
        self.stop_state = StopState()

        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)

        self.stage_plate_offset_x = rospy.get_param("stage_plate_offset_x")
        self.stage_plate_offset_y = rospy.get_param("stage_plate_offset_y")
        self.stage_plate_quat_z = rospy.get_param("stage_plate_quat_z")
        self.stage_plate_quat_w = rospy.get_param("stage_plate_quat_w")

        self.stage_plate_offset_x_error = 0
        self.stage_plate_offset_y_error = 0

        self.robot_origin = PointStamped()
        self.robot_origin.header.frame_id = "Robot"
        self.robot_origin.point.x = 0
        self.robot_origin.point.y = 0
        self.robot_origin.point.z = 0
        self.magnet_origin = PointStamped()
        self.magnet_origin.header.frame_id = "Magnet"
        self.magnet_origin.point.x = 0
        self.magnet_origin.point.y = 0
        self.magnet_origin.point.z = 0
        self.dummy_point = PointStamped()
        self.dummy_point.header.frame_id = "Plate"
        self.dummy_point.point.x = 0
        self.dummy_point.point.y = 0
        self.dummy_point.point.z = 0

        self.position_threshold = 0.01

        self.tries_limit = 4

        self.kf_stage_plate_offset = kf.KalmanFilter()
        self.initialized = True

    def convert_to_plate(self,point):
        point_converted = False
        tries = 0
        point_plate = copy.deepcopy(self.dummy_point)
        self.dummy_point.header.frame_id = "Plate"
        while (not point_converted) and (tries < self.tries_limit):
            tries += 1
            try:
                point_plate = self.tf_listener.transformPoint("Plate",point)
                point_converted = True
            except (tf.LookupException, tf.ConnectivityException):
                pass
        return point_plate

    def stop_state_callback(self,data):
        self.stop_state = copy.deepcopy(data)

    def broadcast(self):
        while not rospy.is_shutdown():
            if self.initialized:
                # try:
                if self.stop_state.RobotStopped:
                    robot_plate = self.convert_to_plate(self.robot_origin)
                    magnet_plate = self.convert_to_plate(self.magnet_origin)
                    # rospy.logwarn("robot_plate.point.x = \n%s" % (str(robot_plate.point.x)))
                    # rospy.logwarn("magnet_plate.point.x = \n%s" % (str(magnet_plate.point.x)))
                    # rospy.logwarn("robot_plate.point.y = \n%s" % (str(robot_plate.point.y)))
                    # rospy.logwarn("magnet_plate.point.y = \n%s" % (str(magnet_plate.point.y)))
                    if (abs(magnet_plate.point.x) < self.position_threshold) and \
                       (abs(magnet_plate.point.y) < self.position_threshold):
                        self.stage_plate_offset_x_error = magnet_plate.point.x - robot_plate.point.x
                        self.stage_plate_offset_y_error = magnet_plate.point.y - robot_plate.point.y
                        # rospy.logwarn("self.stage_plate_offset_x_error = \n%s" % (str(self.stage_plate_offset_x_error)))
                        # rospy.logwarn("self.stage_plate_offset_y_error = \n%s" % (str(self.stage_plate_offset_y_error)))

                        stage_plate_offset_x_adjusted = self.stage_plate_offset_x + self.stage_plate_offset_x_error
                        stage_plate_offset_y_adjusted = self.stage_plate_offset_y + self.stage_plate_offset_y_error
                        # rospy.logwarn("stage_plate_offset_x_adjusted = \n%s" % (str(stage_plate_offset_x_adjusted)))
                        # rospy.logwarn("stage_plate_offset_y_adjusted = \n%s" % (str(stage_plate_offset_y_adjusted)))

                        t = rospy.get_time()
                        (x,y,vx,vy) = self.kf_stage_plate_offset.update((stage_plate_offset_x_adjusted,stage_plate_offset_y_adjusted),t)
                        rospy.logwarn("x = \n%s" % (str(x)))
                        rospy.logwarn("y = \n%s" % (str(y)))
                        self.stage_plate_offset_x = x
                        self.stage_plate_offset_y = y

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
