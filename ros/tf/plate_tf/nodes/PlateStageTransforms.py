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
        now = rospy.Time.now()
        self.tf_listener.waitForTransform("Plate", "Stage", now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("Plate", "Stage", now)
        rospy.logwarn("trans = %s" % (str(trans)))
        rospy.logwarn("rot = %s" % (str(rot)))

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
