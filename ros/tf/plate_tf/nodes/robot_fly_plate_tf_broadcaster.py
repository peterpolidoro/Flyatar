#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf, numpy, math
from geometry_msgs.msg import PoseStamped
from plate_tf.srv import *
import kalman_filter as kf
import stop_walk as sw
import choose_orientation as co
from plate_tf.msg import StopState, InBoundsState, Angles
from pythonmodules import CircleFunctions

class PoseTFConversion:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('RobotImagePose',PoseStamped,self.handle_robot_image_pose)
        self.fly_image_pose_sub = rospy.Subscriber('FlyImagePose',PoseStamped,self.handle_fly_image_pose)

        self.stop_pub = rospy.Publisher('StopState',StopState)
        self.in_bounds_pub = rospy.Publisher('InBoundsState',InBoundsState)
        self.stop_state = StopState()
        self.in_bounds_state = InBoundsState()

        self.kf_fly = kf.KalmanFilter()
        self.kf_robot = kf.KalmanFilter()

        self.sw_fly = sw.StopWalk()
        self.sw_robot = sw.StopWalk()

        self.co_fly = co.ChooseOrientation()
        self.co_robot = co.ChooseOrientation()

        self.in_bounds_radius = rospy.get_param('in_bounds_radius',100)

        self.angles = Angles()
        self.angles_pub = rospy.Publisher('Angles',Angles)
        self.angle_threshold = 0.2

        rospy.wait_for_service('camera_to_plate')
        try:
            self.camera_to_plate = rospy.ServiceProxy('camera_to_plate', PlateCameraConversion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.initialized = True

    def mag_angle_from_x_y(self,vx,vy):
        vel_mag = math.sqrt(vx**2 + vy**2)
        vel_ang = math.atan2(vy,vx)
        return (vel_mag,vel_ang)

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
            quat_plate = tf.transformations.quaternion_from_matrix(T)
            return quat_plate
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError, ValueError):
            return None

    def handle_robot_image_pose(self,msg):
        if self.initialized:
            try:
                Xsrc = [msg.pose.position.x]
                Ysrc = [msg.pose.position.y]
                if (Xsrc is not None) and (Ysrc is not None) and \
                       (0 < len(Xsrc)) and (0 < len(Ysrc)):
                    self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                          (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                          rospy.Time.now(),
                                          "RobotImage",
                                          "Camera")
                    response = self.camera_to_plate(Xsrc,Ysrc)
                    robot_plate_x = response.Xdst[0]
                    robot_plate_y = response.Ydst[0]

                    quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

                    if quat_plate is not None:
                        # rospy.logwarn("robot_plate_a_unfiltered = %s" % (str(tf.transformations.euler_from_quaternion(quat_plate)[2])))
                        robot_plate_a = tf.transformations.euler_from_quaternion(quat_plate)[2]
                        t = msg.header.stamp.to_sec()
                        # (x,y,vx,vy) = self.kf_robot.update((robot_plate_x,robot_plate_y),t)
                        (x,y,a,vx,vy,va) = self.kf_robot.update((robot_plate_x,robot_plate_y,robot_plate_a),t)

                        # rospy.logwarn("robot: x = %s, y = %s, vx = %s, vy = %s" % (x,y,vx,vy))
                        if (vx is not None) and (vy is not None):
                            vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                            robot_stopped = self.sw_robot.classify(vel_mag)
                            # rospy.logwarn("robot_stopped = %s" % (robot_stopped))
                        else:
                            vel_mag = vel_ang = robot_stopped = None

                        if robot_stopped:
                            self.stop_state.RobotStopped = 1
                        else:
                            self.stop_state.RobotStopped = 0

                        self.stop_pub.publish(self.stop_state)

                        if (x is not None) and (y is not None):
                            robot_plate_x = x
                            robot_plate_y = y

                        # if a is not None:
                        #     quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))

                        # Do not flip robot orientation
                        self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
                                              quat_plate,
                                              rospy.Time.now(),
                                              "Robot",
                                              "Plate")

                        robot_dist = math.sqrt(robot_plate_x**2 + robot_plate_y**2)
                        if robot_dist < self.in_bounds_radius:
                            self.in_bounds_state.RobotInBounds = 1
                        else:
                            self.in_bounds_state.RobotInBounds = 0
                        self.in_bounds_pub.publish(self.in_bounds_state)

                    # if vel_ang is not None:
                    #     quat_chosen = self.co_robot.choose_orientation(quat_plate,vel_ang,robot_stopped)
                    # else:
                    #     quat_chosen = None

                    # if quat_chosen is not None:
                    #     self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
                    #                           quat_chosen,
                    #                           rospy.Time.now(),
                    #                           "Robot",
                    #                           "Plate")

            except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
                pass

    def handle_fly_image_pose(self,msg):
        if self.initialized:
            try:
                Xsrc = [msg.pose.position.x]
                Ysrc = [msg.pose.position.y]
                if (Xsrc is not None) and (Ysrc is not None) and \
                       (0 < len(Xsrc)) and (0 < len(Ysrc)):
                    self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                                      (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                                      rospy.Time.now(),
                                                      "FlyImage",
                                                      "Camera")
                    response = self.camera_to_plate(Xsrc,Ysrc)
                    fly_plate_x = response.Xdst[0]
                    fly_plate_y = response.Ydst[0]

                    quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

                    if quat_plate is not None:
                        # rospy.logwarn("fly_plate_a_unfiltered = %s" % (str(tf.transformations.euler_from_quaternion(quat_plate)[2])))
                        fly_plate_a = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])
                        t = msg.header.stamp.to_sec()
                        # (x,y,vx,vy) = self.kf_fly.update((fly_plate_x,fly_plate_y),t)
                        (x,y,a,vx,vy,va) = self.kf_fly.update((fly_plate_x,fly_plate_y,fly_plate_a),t)
                        a = CircleFunctions.mod_angle(a)
                        # rospy.logwarn("fly_plate_a_filtered = %s" % (str(a)))

                        # rospy.logwarn("fly: x = %s, y = %s, vx = %s, vy = %s" % (x,y,vx,vy))
                        if (vx is not None) and (vy is not None):
                            vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                            fly_stopped = self.sw_fly.classify(vel_mag)
                            # rospy.logwarn("fly_stopped = %s" % (fly_stopped))
                        else:
                            vel_mag = vel_ang = fly_stopped = None

                        if fly_stopped:
                            self.stop_state.FlyStopped = 1
                        else:
                            self.stop_state.FlyStopped = 0

                        self.stop_pub.publish(self.stop_state)

                        if (x is not None) and (y is not None):
                            fly_plate_x = x
                            fly_plate_y = y

                        self.angles.Angle = fly_plate_a
                        self.angles.AngleFiltered = a
                        self.angles_pub.publish(self.angles)
                        self.angles.UsingFilteredAngle = 0

                        if a is not None:
                            if abs(CircleFunctions.circle_dist(fly_plate_a,a)) < self.angle_threshold:
                                quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))
                                self.angles.UsingFilteredAngle = 1

                        if vel_ang is not None:
                            quat_chosen = self.co_fly.choose_orientation(quat_plate,vel_ang,fly_stopped)
                        else:
                            quat_chosen = None

                        if quat_chosen is not None:
                            self.tf_broadcaster.sendTransform((fly_plate_x, fly_plate_y, 0),
                                                  quat_chosen,
                                                  rospy.Time.now(),
                                                  "Fly",
                                                  "Plate")

                        fly_dist = math.sqrt(fly_plate_x**2 + fly_plate_y**2)
                        if fly_dist < self.in_bounds_radius:
                            self.in_bounds_state.FlyInBounds = 1
                        else:
                            self.in_bounds_state.FlyInBounds = 0
                        self.in_bounds_pub.publish(self.in_bounds_state)

            except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
                pass

if __name__ == '__main__':
    rospy.init_node('robot_fly_plate_tf_broadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()

