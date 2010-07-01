#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
import numpy
import tf
from plate_tf.srv import *
from geometry_msgs.msg import PointStamped

class Transforms:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.transforms_initialized = False
        while not self.transforms_initialized:
            try:
                (self.trans1,self.rot1) = self.tf_listener.lookupTransform("Plate", "Stage", rospy.Time.now())
                (self.trans2,self.rot2) = self.tf_listener.lookupTransform("Stage", "Plate", rospy.Time.now())
                self.transforms_initialized = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.logwarn("trans1 = %s" % (str(self.trans1)))
        rospy.logwarn("rot1 = %s" % (str(self.rot1)))
        rospy.logwarn("trans2 = %s" % (str(self.trans2)))
        rospy.logwarn("rot2 = %s" % (str(self.rot2)))

        self.T1 = tf.transformations.translation_matrix(self.trans1)
        self.R1 = tf.transformations.quaternion_matrix(self.rot1)
        self.M11 = tf.transformations.concatenate_matrices(self.R1, self.T1)
        self.M11inv = numpy.linalg.inv(self.M11)
        self.M12 = tf.transformations.concatenate_matrices(self.T1, self.R1)
        self.M12inv = numpy.linalg.inv(self.M12)

        self.T2 = tf.transformations.translation_matrix(self.trans2)
        self.R2 = tf.transformations.quaternion_matrix(self.rot2)
        self.M21 = tf.transformations.concatenate_matrices(self.R2, self.T2)
        self.M21inv = numpy.linalg.inv(self.M21)
        self.M22 = tf.transformations.concatenate_matrices(self.T2, self.R2)
        self.M22inv = numpy.linalg.inv(self.M22)
        # rospy.logwarn("T = \n%s" % (str(self.T)))
        # rospy.logwarn("R = \n%s" % (str(self.R)))
        Xsrc = [0,10]
        Ysrc = [0,10]
        Zsrc = [0,0]
        Hsrc = [1,1]
        plate_points = numpy.array([Xsrc,Ysrc,Zsrc,Hsrc])
        # rospy.logwarn("M = \n%s" % (str(self.M)))
        rospy.logwarn("plate_points = \n%s" % (str(plate_points)))
        stage_points = numpy.dot(self.M11,plate_points)
        rospy.logwarn("stage_points11 = \n%s" % (str(stage_points)))
        stage_points = numpy.dot(self.M12,plate_points)
        rospy.logwarn("stage_points12 = \n%s" % (str(stage_points)))
        stage_points = numpy.dot(self.M21,plate_points)
        rospy.logwarn("stage_points21 = \n%s" % (str(stage_points)))
        stage_points = numpy.dot(self.M22,plate_points)
        rospy.logwarn("stage_points22 = \n%s" % (str(stage_points)))

        plate_point = PointStamped()
        plate_point.header.frame_id = "Plate"
        plate_point.point.x = 0
        plate_point.point.y = 0
        plate_point.point.z = 0
        stage_point = self.tf_listener.transformPoint("Stage",plate_point)
        rospy.logwarn("plate_point1.x = \n%s" % (str(plate_point.point.x)))
        rospy.logwarn("plate_point1.y = \n%s" % (str(plate_point.point.y)))
        rospy.logwarn("stage_point1.x = \n%s" % (str(stage_point.point.x)))
        rospy.logwarn("stage_point1.y = \n%s" % (str(stage_point.point.y)))
        plate_point.point.x = 10
        plate_point.point.y = 10
        stage_point = self.tf_listener.transformPoint("Stage",plate_point)
        rospy.logwarn("plate_point2.x = \n%s" % (str(plate_point.point.x)))
        rospy.logwarn("plate_point2.y = \n%s" % (str(plate_point.point.y)))
        rospy.logwarn("stage_point2.x = \n%s" % (str(stage_point.point.x)))
        rospy.logwarn("stage_point2.y = \n%s" % (str(stage_point.point.y)))

        Xsrc = [100,50]
        Ysrc = [100,150]
        Zsrc = [0,0]
        Hsrc = [1,1]
        stage_points = numpy.array([Xsrc,Ysrc,Zsrc,Hsrc])
        # rospy.logwarn("M = \n%s" % (str(self.M)))
        rospy.logwarn("stage_points = \n%s" % (str(stage_points)))
        plate_points = numpy.dot(self.M22inv,stage_points)
        rospy.logwarn("plate_points22inv = \n%s" % (str(plate_points)))

        stage_point.point.x = 100
        stage_point.point.y = 100
        plate_point = self.tf_listener.transformPoint("Plate",stage_point)
        rospy.logwarn("stage_point1.x = \n%s" % (str(stage_point.point.x)))
        rospy.logwarn("stage_point1.y = \n%s" % (str(stage_point.point.y)))
        rospy.logwarn("plate_point1.x = \n%s" % (str(plate_point.point.x)))
        rospy.logwarn("plate_point1.y = \n%s" % (str(plate_point.point.y)))
        stage_point.point.x = 50
        stage_point.point.y = 150
        plate_point = self.tf_listener.transformPoint("Plate",stage_point)
        rospy.logwarn("stage_point2.x = \n%s" % (str(stage_point.point.x)))
        rospy.logwarn("stage_point2.y = \n%s" % (str(stage_point.point.y)))
        rospy.logwarn("plate_point2.x = \n%s" % (str(plate_point.point.x)))
        rospy.logwarn("plate_point2.y = \n%s" % (str(plate_point.point.y)))

    def plate_to_stage(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*point_count
        plate_points = numpy.array([Xsrc,Ysrc,Zsrc])
        stage_points = numpy.dot(self.M,plate_points)
        Xdst = stage_points[0,:]
        Ydst = stage_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

    def stage_to_plate(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [0]*point_count
        stage_points = numpy.array([Xsrc,Ysrc,Zsrc])
        plate_points = numpy.dot(self.Minv,stage_points)

        Xdst = plate_points[0,:]
        Ydst = plate_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

def plate_stage_transforms_server():
    rospy.init_node('plate_stage_transforms_server')
    transforms = Transforms()
    s_ps = rospy.Service('plate_to_stage', PlateStageConversion, transforms.plate_to_stage)
    s_sp = rospy.Service('stage_to_plate', PlateStageConversion, transforms.stage_to_plate)

if __name__ == "__main__":
    plate_stage_transforms_server()
    while not rospy.is_shutdown():
        rospy.spin()
