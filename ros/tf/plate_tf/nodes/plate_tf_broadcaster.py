#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf, numpy
from geometry_msgs.msg import PoseStamped
from plate_tf.srv import *

class PoseTFConversion:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('RobotImagePose',PoseStamped,self.handle_robot_image_pose)
        self.fly_image_pose_sub = rospy.Subscriber('FlyImagePose',PoseStamped,self.handle_fly_image_pose)

        rospy.wait_for_service('camera_to_plate')
        try:
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def quaternion_camera_to_plate(self,quat):
        # Must be cleverer way to calculate this using quaternion math...
        R = tf.transformations.quaternion_matrix(quat)
        points_camera = numpy.array(\
            [[0,  1, -1,  0,  0,  1, -1],
             [0,  0,  0,  1, -1,  1, -1],
             [0,  0,  0,  0,  0,  0,  0],
             [1,  1,  1,  1,  1,  1,  1]])
        rospy.logwarn("points_camera = \n%s", str(points_camera))
        points_camera_rotated = numpy.dot(R,points_camera)
        rospy.logwarn("points_camera_rotated = \n%s", str(points_camera_rotated))
        try:
            Xsrc = list(points_camera[0,:])
            Ysrc = list(points_camera[1,:])
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_x = list(response.Xdst)
            points_plate_y = list(response.Ydst)
            z = [0]*len(points_plate_x)
            w = [1]*len(points_plate_y)
            points_plate = numpy.array([points_plate_x,points_plate_y,z,w])
            rospy.logwarn("points_plate = \n%s", str(points_plate))

            Xsrc = list(points_camera_rotated[0,:])
            Ysrc = list(points_camera_rotated[1,:])
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_rotated_x = list(response.Xdst)
            points_plate_rotated_y = list(response.Ydst)
            points_plate_rotated = numpy.array([points_plate_rotated_x,points_plate_rotated_y,z,w])
            rospy.logwarn("points_plate_rotated = \n%s", str(points_plate_rotated))
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass

    def handle_robot_image_pose(self,msg):
        try:
            Xsrc = [msg.pose.position.x]
            Ysrc = [msg.pose.position.y]
            self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                  (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                  rospy.Time.now(),
                                  "RobotImage",
                                  "Camera")

            response = self.camera_to_plate(Xsrc,Ysrc)
            robot_plate_x = response.Xdst[0]
            robot_plate_y = response.Ydst[0]
            self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "Robot",
                                  "Plate")
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass

    def handle_fly_image_pose(self,msg):
        try:
            Xsrc = [msg.pose.position.x]
            Ysrc = [msg.pose.position.y]
            self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                  (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                  rospy.Time.now(),
                                  "FlyImage",
                                  "Camera")

            self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

            response = self.camera_to_plate(Xsrc,Ysrc)
            fly_plate_x = response.Xdst[0]
            fly_plate_y = response.Ydst[0]
            self.tf_broadcaster.sendTransform((fly_plate_x, fly_plate_y, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, 1),
                                  rospy.Time.now(),
                                  "Fly",
                                  "Plate")
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass

if __name__ == '__main__':
    rospy.init_node('plate_tf_broadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()
