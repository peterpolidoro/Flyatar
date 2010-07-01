#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('plate_tf')
import rospy
import numpy
import tf
from plate_tf.srv import *

class Transforms:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.transforms_initialized = False
        while not self.transforms_initialized:
            try:
                (self.trans,self.rot) = self.tf_listener.lookupTransform("Plate", "Stage", rospy.Time.now())
                self.transforms_initialized = True
            except (tf.LookupException, tf.ConnectivityException):
                pass

        rospy.logwarn("trans = %s" % (str(self.trans)))
        rospy.logwarn("rot = %s" % (str(self.rot)))

        self.T = tf.transformations.translation_matrix(self.trans)
        self.R = tf.transformations.quaternion_matrix(self.rot)
        self.M1 = tf.transformations.concatenate_matrices(self.R, self.T)
        self.M2 = tf.transformations.concatenate_matrices(self.T, self.R)
        rospy.logwarn("T = \n%s" % (str(self.T)))
        rospy.logwarn("R = \n%s" % (str(self.R)))
        rospy.logwarn("M1 = \n%s" % (str(self.M1)))
        rospy.logwarn("M2 = \n%s" % (str(self.M2)))

        # (intrinsic_matrix,distortion_coeffs) = StageParameters.intrinsic("rect")
        # (rvec,tvec) = StageParameters.extrinsic("plate")
        # intrinsic_matrix = cvNumpy.mat_to_array(intrinsic_matrix)
        # rvec = cvNumpy.mat_to_array(rvec).squeeze()
        # tvec = cvNumpy.mat_to_array(tvec).squeeze()

        # rvec_angle = numpy.linalg.norm(rvec)
        # R = tf.transformations.rotation_matrix(rvec_angle,rvec)
        # T = tf.transformations.translation_matrix(tvec)

        # Wsub = numpy.zeros((3,3))
        # Wsub[:,:-1] = R[:-1,:-2]
        # Wsub[:,-1] = T[:-1,-1]

        # M = intrinsic_matrix
        # # Makes with respect to Stage coordinate system instead of Undistorted
        # M[:-1,-1] = 0

        # Hinv = numpy.dot(M,Wsub)
        # Hinv = Hinv/Hinv[-1,-1]
        # H = numpy.linalg.inv(Hinv)
        # self.Hinv = Hinv
        # self.H = H
        # # rospy.logwarn("H = \n%s", str(H))


    def plate_to_stage(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        plate_points = numpy.array([Xsrc,Ysrc,Zsrc])
        stage_points = numpy.dot(self.Hinv,plate_points)
        Xdst = stage_points[0,:]
        Ydst = stage_points[1,:]
        return {'Xdst': Xdst,
                'Ydst': Ydst}

    def stage_to_plate(self,req):
        point_count = min(len(req.Xsrc),len(req.Ysrc))
        Xsrc = list(req.Xsrc)
        Ysrc = list(req.Ysrc)
        Zsrc = [1]*point_count
        stage_points = numpy.array([Xsrc,Ysrc,Zsrc])
        plate_points = numpy.dot(self.H,stage_points)

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
