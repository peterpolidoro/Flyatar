#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf, numpy
from geometry_msgs.msg import PoseStamped
from plate_tf.srv import *
import kalman_filter as kf

class PoseTFConversion:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('RobotImagePose',PoseStamped,self.handle_robot_image_pose)
        self.fly_image_pose_sub = rospy.Subscriber('FlyImagePose',PoseStamped,self.handle_fly_image_pose)

        self.kf_fly = kf.KalmanFilter()

        rospy.wait_for_service('camera_to_plate')
        try:
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def quaternion_camera_to_plate(self,quat):
        # Must be cleverer way to calculate this using quaternion math...
        R = tf.transformations.quaternion_matrix(quat)
        scale_factor = 100
        points_camera = numpy.array(\
            [[0,  1, -1,  0,  0,  1, -1],
             [0,  0,  0,  1, -1,  1, -1],
             [0,  0,  0,  0,  0,  0,  0],
             [1,  1,  1,  1,  1,  1,  1]])
        points_camera = points_camera*scale_factor
        # rospy.logwarn("points_camera = \n%s", str(points_camera))
        points_camera_rotated = numpy.dot(R,points_camera)
        # rospy.logwarn("points_camera_rotated = \n%s", str(points_camera_rotated))
        try:
            Xsrc = list(points_camera[0,:])
            Ysrc = list(points_camera[1,:])
            # rospy.logwarn("Xsrc = %s", str(Xsrc))
            # rospy.logwarn("Ysrc = %s", str(Ysrc))
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_x = list(response.Xdst)
            points_plate_y = list(response.Ydst)
            z = [0]*len(points_plate_x)
            w = [1]*len(points_plate_y)
            points_plate = numpy.array([points_plate_x,points_plate_y,z,w])
            points_plate = numpy.append(points_plate,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate = \n%s", str(points_plate))

            Xsrc = list(points_camera_rotated[0,:])
            Ysrc = list(points_camera_rotated[1,:])
            response = self.camera_to_plate(Xsrc,Ysrc)
            points_plate_rotated_x = list(response.Xdst)
            points_plate_rotated_y = list(response.Ydst)
            # rospy.logwarn("points_plate_rotated_x = %s",str(points_plate_rotated_x))
            # rospy.logwarn("points_plate_rotated_y = %s",str(points_plate_rotated_y))
            # rospy.logwarn("z = %s",str(z))
            # rospy.logwarn("w = %s",str(w))
            points_plate_rotated = numpy.array([points_plate_rotated_x,points_plate_rotated_y,z,w])
            points_plate_rotated = numpy.append(points_plate_rotated,[[0,0],[0,0],[scale_factor,-scale_factor],[1,1]],axis=1)
            # rospy.logwarn("points_plate_rotated = \n%s", str(points_plate_rotated))
            T = tf.transformations.superimposition_matrix(points_plate_rotated,points_plate)
            # rospy.logwarn("T = \n%s", str(T))
            # al, be, ga = tf.transformations.euler_from_matrix(T, 'rxyz')
            # rospy.logwarn("ga = %s" % str(ga*180/numpy.pi))
            quat_converted = tf.transformations.quaternion_from_matrix(T)
            return quat_converted
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError, ValueError):
            return None

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

            quat_converted = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))
            if quat_converted is not None:
                self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
                                      quat_converted,
                                      rospy.Time.now(),
                                      "Robot",
                                      "Plate")
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
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

            response = self.camera_to_plate(Xsrc,Ysrc)
            fly_plate_x = response.Xdst[0]
            fly_plate_y = response.Ydst[0]

            t = msg.header.stamp.to_seconds()
            self.kf_fly.update((fly_plate_x,fly_plate_y),t)

            quat_converted = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))
            if quat_converted is not None:
                self.tf_broadcaster.sendTransform((fly_plate_x, fly_plate_y, 0),
                                      quat_converted,
                                      rospy.Time.now(),
                                      "Fly",
                                      "Plate")
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
            pass

if __name__ == '__main__':
    rospy.init_node('plate_tf_broadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()
