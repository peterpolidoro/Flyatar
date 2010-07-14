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
from plate_tf.msg import StopState, InBoundsState, FilteredData
from pythonmodules import CircleFunctions

class PoseTFConversion:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('ImagePose/Robot',PoseStamped,self.handle_robot_image_pose)
        self.fly_image_pose_sub = rospy.Subscriber('ImagePose/Fly',PoseStamped,self.handle_fly_image_pose)

        self.fly_stop_pub = rospy.Publisher('StopState/Fly',StopState)
        self.fly_in_bounds_pub = rospy.Publisher('InBoundsState/Fly',InBoundsState)
        self.robot_stop_pub = rospy.Publisher('StopState/Robot',StopState)
        self.robot_in_bounds_pub = rospy.Publisher('InBoundsState/Robot',InBoundsState)

        self.fly_stop_state = StopState()
        self.fly_in_bounds_state = InBoundsState()
        self.robot_stop_state = StopState()
        self.robot_in_bounds_state = InBoundsState()

        self.kf_fly = kf.KalmanFilter()
        self.kf_robot = kf.KalmanFilter()

        self.sw_fly = sw.StopWalk()
        self.sw_robot = sw.StopWalk()

        self.co_fly = co.ChooseOrientation()
        self.co_robot = co.ChooseOrientation()

        self.in_bounds_radius = rospy.get_param('in_bounds_radius',100)

        self.robot_x_filtered_data = FilteredData()
        self.robot_x_filtered_data_pub = rospy.Publisher('FilteredData/RobotX',FilteredData)
        self.robot_y_filtered_data = FilteredData()
        self.robot_y_filtered_data_pub = rospy.Publisher('FilteredData/RobotY',FilteredData)
        self.robot_a_filtered_data = FilteredData()
        self.robot_a_filtered_data_pub = rospy.Publisher('FilteredData/RobotAngle',FilteredData)

        self.fly_x_filtered_data = FilteredData()
        self.fly_x_filtered_data_pub = rospy.Publisher('FilteredData/FlyX',FilteredData)
        self.fly_y_filtered_data = FilteredData()
        self.fly_y_filtered_data_pub = rospy.Publisher('FilteredData/FlyY',FilteredData)
        self.fly_a_filtered_data = FilteredData()
        self.fly_a_filtered_data_pub = rospy.Publisher('FilteredData/FlyAngle',FilteredData)

        self.angle_threshold = 0.2
        self.position_threshold = 2

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

    def image_pose_handler(self,object_name,msg):
        if "Fly" in object_name:
            image_frame_name = "FlyImage"
            frame_name = "Fly"
            kf = self.kf_fly
            sw = self.sw_fly
            co = self.co_fly
            stop_pub = self.fly_stop_pub
            in_bounds_pub = self.fly_in_bounds_pub
            stop_state = self.fly_stop_state
            in_bounds_state = self.fly_in_bounds_state
            x_filtered_data = self.fly_x_filtered_data
            x_filtered_data_pub = self.fly_x_filtered_data_pub
            y_filtered_data = self.fly_y_filtered_data
            y_filterd_data_pub = self.fly_y_filtered_data_pub
            a_filtered_data = self.fly_a_filtered_data
            a_filtered_data_pub = self.fly_a_filtered_data_pub
        else:
            image_frame_name = "RobotImage"
            frame_name = "Robot"
            kf = self.kf_robot
            sw = self.sw_robot
            co = self.co_robot
            stop_pub = self.robot_stop_pub
            in_bounds_pub = self.robot_in_bounds_pub
            stop_state = self.robot_stop_state
            in_bounds_state = self.robot_in_bounds_state
            x_filtered_data = self.robot_x_filtered_data
            x_filtered_data_pub = self.robot_x_filtered_data_pub
            y_filtered_data = self.robot_y_filtered_data
            y_filterd_data_pub = self.robot_y_filtered_data_pub
            a_filtered_data = self.robot_a_filtered_data
            a_filtered_data_pub = self.robot_a_filtered_data_pub

        try:
            Xsrc = [msg.pose.position.x]
            Ysrc = [msg.pose.position.y]
            if (Xsrc is not None) and (Ysrc is not None) and \
                   (0 < len(Xsrc)) and (0 < len(Ysrc)):
                self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                                  (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                                                  rospy.Time.now(),
                                                  image_frame_name,
                                                  "Camera")
                response = self.camera_to_plate(Xsrc,Ysrc)
                x_plate = response.Xdst[0]
                y_plate = response.Ydst[0]

                quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

                if quat_plate is not None:
                    a_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])
                    t = msg.header.stamp.to_sec()
                    (x,y,vx,vy) = kf.update((x_plate,y_plate),t)

                    if (vx is not None) and (vy is not None):
                        vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                        stopped = sw.classify(vel_mag)
                    else:
                        vel_mag = vel_ang = stopped = None

                    if stopped:
                        stop_state.Stopped = 1
                    else:
                        stop_state.Stopped = 0

                    stop_pub.publish(stop_state)

                    x_filtered_data.UnFiltered = x_plate
                    y_filtered_data.UnFiltered = y_plate
                    x_filtered_data.UsingFiltered = 0
                    y_filtered_data.UsingFiltered = 0

                    if (x is not None) and (y is not None) and \
                           (abs(x_plate - x) < self.position_threshold) and \
                           (abs(y_plate - y) < self.position_threshold):
                        x_plate = x
                        y_plate = y
                        x_filtered_data.UsingFiltered = 1
                        y_filtered_data.UsingFiltered = 1
                        x_filtered_data.Filtered = x
                        y_filtered_data.Filtered = y

                    a_filtered_data.Unfiltered = a_plate
                    # a_filtered_data.Filtered = a
                    a_filtered_data.UsingFiltered = 0

                    if a is not None:
                        if abs(CircleFunctions.circle_dist(a_plate,a)) < self.angle_threshold:
                            quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))
                            a_filtered_data.UsingFiltered = 1

                    x_filtered_data_pub.publish(x_filtered_data)
                    y_filtered_data_pub.publish(y_filtered_data)
                    a_filtered_data_pub.publish(a_filtered_data)

                    if vel_ang is not None:
                        quat_chosen = co.choose_orientation(quat_plate,vel_ang,stopped)
                    else:
                        quat_chosen = None

                    if quat_chosen is not None:
                        self.tf_broadcaster.sendTransform((x_plate, y_plate, 0),
                                              quat_chosen,
                                              rospy.Time.now(),
                                              frame_name,
                                              "Plate")

                    dist = math.sqrt(x_plate**2 + y_plate**2)
                    if dist < self.in_bounds_radius:
                        in_bounds_state.InBounds = 1
                    else:
                        in_bounds_state.InBounds = 0
                    in_bounds_pub.publish(in_bounds_state)

        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
        # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
            pass


    def handle_robot_image_pose(self,msg):
        if self.initialized:
            self.image_pose_handler("Robot",msg)
            # try:
            #     Xsrc = [msg.pose.position.x]
            #     Ysrc = [msg.pose.position.y]
            #     if (Xsrc is not None) and (Ysrc is not None) and \
            #            (0 < len(Xsrc)) and (0 < len(Ysrc)):
            #         self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
            #                               (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
            #                               rospy.Time.now(),
            #                               "RobotImage",
            #                               "Camera")
            #         response = self.camera_to_plate(Xsrc,Ysrc)
            #         robot_plate_x = response.Xdst[0]
            #         robot_plate_y = response.Ydst[0]

            #         quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

            #         if quat_plate is not None:
            #             # rospy.logwarn("robot_plate_a_unfiltered = %s" % (str(tf.transformations.euler_from_quaternion(quat_plate)[2])))
            #             robot_plate_a = tf.transformations.euler_from_quaternion(quat_plate)[2]
            #             t = msg.header.stamp.to_sec()
            #             # (x,y,vx,vy) = self.kf_robot.update((robot_plate_x,robot_plate_y),t)
            #             (x,y,a,vx,vy,va) = self.kf_robot.update((robot_plate_x,robot_plate_y,robot_plate_a),t)

            #             # rospy.logwarn("robot: x = %s, y = %s, vx = %s, vy = %s" % (x,y,vx,vy))
            #             if (vx is not None) and (vy is not None):
            #                 vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
            #                 robot_stopped = self.sw_robot.classify(vel_mag)
            #                 # rospy.logwarn("robot_stopped = %s" % (robot_stopped))
            #             else:
            #                 vel_mag = vel_ang = robot_stopped = None

            #             if robot_stopped:
            #                 self.stop_state.RobotStopped = 1
            #             else:
            #                 self.stop_state.RobotStopped = 0

            #             self.stop_pub.publish(self.stop_state)

            #             if (x is not None) and (y is not None) and \
            #                    (abs(robot_plate_x - x) < self.position_threshold) and \
            #                    (abs(robot_plate_y - y) < self.position_threshold):
            #                 robot_plate_x = x
            #                 robot_plate_y = y

            #             # if a is not None:
            #             #     quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))

            #             # Do not flip robot orientation
            #             self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
            #                                   quat_plate,
            #                                   rospy.Time.now(),
            #                                   "Robot",
            #                                   "Plate")

            #             robot_dist = math.sqrt(robot_plate_x**2 + robot_plate_y**2)
            #             if robot_dist < self.in_bounds_radius:
            #                 self.in_bounds_state.RobotInBounds = 1
            #             else:
            #                 self.in_bounds_state.RobotInBounds = 0
            #             self.in_bounds_pub.publish(self.in_bounds_state)

            #         # if vel_ang is not None:
            #         #     quat_chosen = self.co_robot.choose_orientation(quat_plate,vel_ang,robot_stopped)
            #         # else:
            #         #     quat_chosen = None

            #         # if quat_chosen is not None:
            #         #     self.tf_broadcaster.sendTransform((robot_plate_x, robot_plate_y, 0),
            #         #                           quat_chosen,
            #         #                           rospy.Time.now(),
            #         #                           "Robot",
            #         #                           "Plate")

            # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            # # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
            #     pass

    def handle_fly_image_pose(self,msg):
        if self.initialized:
            self.image_pose_handler("Fly",msg)
            # try:
            #     Xsrc = [msg.pose.position.x]
            #     Ysrc = [msg.pose.position.y]
            #     if (Xsrc is not None) and (Ysrc is not None) and \
            #            (0 < len(Xsrc)) and (0 < len(Ysrc)):
            #         self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
            #                                           (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
            #                                           rospy.Time.now(),
            #                                           "FlyImage",
            #                                           "Camera")
            #         response = self.camera_to_plate(Xsrc,Ysrc)
            #         fly_plate_x = response.Xdst[0]
            #         fly_plate_y = response.Ydst[0]

            #         quat_plate = self.quaternion_camera_to_plate((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))

            #         if quat_plate is not None:
            #             # rospy.logwarn("fly_plate_a_unfiltered = %s" % (str(tf.transformations.euler_from_quaternion(quat_plate)[2])))
            #             fly_plate_a = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])
            #             t = msg.header.stamp.to_sec()
            #             # (x,y,vx,vy) = self.kf_fly.update((fly_plate_x,fly_plate_y),t)
            #             (x,y,a,vx,vy,va) = self.kf_fly.update((fly_plate_x,fly_plate_y,fly_plate_a),t)
            #             a = CircleFunctions.mod_angle(a)
            #             # rospy.logwarn("fly_plate_a_filtered = %s" % (str(a)))

            #             # rospy.logwarn("fly: x = %s, y = %s, vx = %s, vy = %s" % (x,y,vx,vy))
            #             if (vx is not None) and (vy is not None):
            #                 vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
            #                 fly_stopped = self.sw_fly.classify(vel_mag)
            #                 # rospy.logwarn("fly_stopped = %s" % (fly_stopped))
            #             else:
            #                 vel_mag = vel_ang = fly_stopped = None

            #             if fly_stopped:
            #                 self.stop_state.FlyStopped = 1
            #             else:
            #                 self.stop_state.FlyStopped = 0

            #             self.stop_pub.publish(self.stop_state)

            #             self.filtered_data.UsingFlyXFiltered = 0
            #             self.filtered_data.UsingFlyYFiltered = 0

            #             if (x is not None) and (y is not None) and \
            #                    (abs(fly_plate_x - x) < self.position_threshold) and \
            #                    (abs(fly_plate_y - y) < self.position_threshold):
            #                 fly_plate_x = x
            #                 fly_plate_y = y
            #                 self.filtered_data.UsingFlyXFiltered = 1
            #                 self.filtered_data.UsingFlyYFiltered = 1

            #             self.filtered_data.FlyAngle = fly_plate_a
            #             self.filtered_data.FlyAngleFiltered = a
            #             self.filtered_data.UsingFlyAngleFiltered = 0

            #             self.filtered_data.FlyAngle = fly_plate_a
            #             self.filtered_data.FlyAngleFiltered = a
            #             self.filtered_data.UsingFlyAngleFiltered = 0

            #             if a is not None:
            #                 if abs(CircleFunctions.circle_dist(fly_plate_a,a)) < self.angle_threshold:
            #                     quat_plate = tf.transformations.quaternion_about_axis(a, (0,0,1))
            #                     self.filtered_data.UsingFlyAngleFiltered = 1

            #             self.filtered_data_pub.publish(self.filtered_data)

            #             if vel_ang is not None:
            #                 quat_chosen = self.co_fly.choose_orientation(quat_plate,vel_ang,fly_stopped)
            #             else:
            #                 quat_chosen = None

            #             if quat_chosen is not None:
            #                 self.tf_broadcaster.sendTransform((fly_plate_x, fly_plate_y, 0),
            #                                       quat_chosen,
            #                                       rospy.Time.now(),
            #                                       "Fly",
            #                                       "Plate")

            #             fly_dist = math.sqrt(fly_plate_x**2 + fly_plate_y**2)
            #             if fly_dist < self.in_bounds_radius:
            #                 self.in_bounds_state.FlyInBounds = 1
            #             else:
            #                 self.in_bounds_state.FlyInBounds = 0
            #             self.in_bounds_pub.publish(self.in_bounds_state)

            # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            # # except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException, AttributeError):
            #     pass

if __name__ == '__main__':
    rospy.init_node('robot_fly_plate_tf_broadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()

