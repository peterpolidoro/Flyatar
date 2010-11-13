#!/usr/bin/env python
import roslib
roslib.load_manifest('plate_tf')
import rospy

import tf, numpy, math
from geometry_msgs.msg import PoseStamped,Pose,PoseArray
from plate_tf.srv import *
import filters
import stop_walk as sw
import choose_orientation as co
from plate_tf.msg import ImagePose
from plate_tf.msg import RobotFlyKinematics
import plate_tf.msg
from pythonmodules import CircleFunctions

class PoseTFConversion:
    def __init__(self):
        self.initialized = False
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot_image_pose_sub = rospy.Subscriber('ImagePose',ImagePose,self.handle_image_pose)

        self.fly_image_pose = Pose()

        self.robot_fly_kinematics = RobotFlyKinematics()
        self.robot_fly_kinematics_pub = rospy.Publisher('RobotFlyKinematics', RobotFlyKinematics)

        # Robot Info
        self.robot_visible = bool(rospy.get_param("robot_visible","true"))

        self.kf_fly = filters.KalmanFilter()
        self.kf_robot = filters.KalmanFilter()
        self.lpf_fly_angle = filters.LowPassFilter()
        self.lpf_robot_angle = filters.LowPassFilter()
        self.fly_angle_previous = [None]
        self.robot_angle_previous = [None]

        self.fly_t_previous = None
        self.robot_t_previous = None

        self.sw_fly = sw.StopWalk()
        self.sw_robot = sw.StopWalk()

        self.co_fly = co.ChooseOrientation()
        self.co_robot = co.ChooseOrientation()

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
            lpf_angle = self.lpf_fly_angle
            a_prev = self.fly_angle_previous
            kinematics_state = self.robot_fly_kinematics.fly_kinematics
            stop_state = self.robot_fly_kinematics.fly_stopped
            t_prev = self.fly_t_previous

        else:
            image_frame_name = "RobotImage"
            frame_name = "Robot"
            kf = self.kf_robot
            sw = self.sw_robot
            co = self.co_robot
            lpf_angle = self.lpf_robot_angle
            a_prev = self.robot_angle_previous
            kinematics_state = self.robot_fly_kinematics.robot_kinematics
            stop_state = self.robot_fly_kinematics.robot_stopped
            t_prev = self.robot_t_previous

        try:
            if (("Robot" in object_name) and self.robot_visible) or ("Fly" in object_name):
                Xsrc = [msg.position.x]
                Ysrc = [msg.position.y]
                if (Xsrc is not None) and (Ysrc is not None) and \
                       (0 < len(Xsrc)) and (0 < len(Ysrc)):
                    self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, 0),
                                                      (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),
                                                      self.robot_fly_kinematics.header.stamp,
                                                      # rospy.Time.now(),
                                                      image_frame_name,
                                                      "Camera")
                    response = self.camera_to_plate(Xsrc,Ysrc)
                    x_plate = response.Xdst[0]
                    y_plate = response.Ydst[0]

                    quat_plate = self.quaternion_camera_to_plate((msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w))
            elif ("Robot" in object_name) and (not self.robot_visible):
                (trans,quat_plate) = self.tf_listener.lookupTransform('/Plate', '/Magnet', rospy.Time(0))
                x_plate = trans[0]
                y_plate = trans[1]

            if quat_plate is not None:
                rospy.logwarn("x_plate = %s" % (str(x_plate)))
                rospy.logwarn("y_plate = %s" % (str(y_plate)))
                rospy.logwarn("quat_plate = %s" % (str(quat_plate)))
                t = self.robot_fly_kinematics.header.stamp.to_sec()
                (x,y,vx,vy) = kf.update((x_plate,y_plate),t)

                if (vx is not None) and (vy is not None):
                    vel_mag,vel_ang = self.mag_angle_from_x_y(vx,vy)
                    stopped = sw.classify(vel_mag)
                    kinematics_state.velocity.x = vx
                    kinematics_state.velocity.y = vy
                else:
                    vel_mag = vel_ang = stopped = None

                if (stopped is not None):
                    stop_state.stopped = stopped

                if (x is not None) and (y is not None) and \
                       (abs(x_plate - x) < self.position_threshold) and \
                       (abs(y_plate - y) < self.position_threshold):
                    kinematics_state.position.x = x
                    kinematics_state.position.y = y
                    x_plate = x
                    y_plate = y
                else:
                    kinematics_state.position.x = x_plate
                    kinematics_state.position.y = y_plate

                if vel_ang is not None:
                    quat_chosen = co.choose_orientation(quat_plate,vel_ang,stopped,a_prev[0])
                else:
                    quat_chosen = None

                if "Robot" in object_name:
                    quat_chosen = quat_plate

                if quat_chosen is not None:
                    a_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_chosen)[2])
                else:
                    a_plate = CircleFunctions.mod_angle(tf.transformations.euler_from_quaternion(quat_plate)[2])

                a_plate = CircleFunctions.unwrap_angle(a_plate,a_prev[0])

                a = lpf_angle.update(a_plate,t)

                if a is not None:
                    kinematics_state.position.theta = a
                    a_mod = CircleFunctions.mod_angle(a)
                    quat_plate = tf.transformations.quaternion_about_axis(a_mod, (0,0,1))
                elif a_plate is not None:
                    kinematics_state.position.theta = a_plate
                    if t_prev is not None:
                        kinematics_state.velocity.theta = (a_plate - a_prev[0])/(t - t_prev)
                    else:
                        kinematics_state.velocity.theta = 0
                else:
                    kinematics_state.position.theta = 0
                    kinematics_state.velocity.theta = 0

                if vel_ang is not None:
                    quat_chosen = co.choose_orientation(quat_plate,vel_ang,stopped,a_prev[0])
                else:
                    quat_chosen = None

                a_prev[0] = a

                if "Robot" in object_name:
                    quat_chosen = quat_plate

                if quat_chosen is not None:
                    self.tf_broadcaster.sendTransform((x_plate, y_plate, 0),
                                                      quat_chosen,
                                                      self.robot_fly_kinematics.header.stamp,
                                                      # rospy.Time.now(),
                                                      frame_name,
                                                      "Plate")

        except (tf.LookupException, tf.ConnectivityException, rospy.ServiceException):
            pass

    def handle_image_pose(self,msg):
        if self.initialized:
            self.robot_fly_kinematics.header = msg.header
            self.robot_fly_kinematics.header.frame_id = "Plate"
            self.image_pose_handler("Robot",msg.robot_image_pose)
            fly_count = len(msg.fly_image_poses)
            # rospy.logwarn("fly count = %s" % (str(fly_count)))
            # Fix at some point to properly account for multiple flies
            if fly_count == 1:
                self.fly_image_pose = msg.fly_image_poses[0]
                self.image_pose_handler("Fly",self.fly_image_pose)
            elif 1 < fly_count:
                self.fly_image_pose = msg.fly_image_poses[0]
                self.image_pose_handler("Fly",self.fly_image_pose)
            self.robot_fly_kinematics_pub.publish(self.robot_fly_kinematics)

if __name__ == '__main__':
    rospy.init_node('RobotFlyTFBroadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()

