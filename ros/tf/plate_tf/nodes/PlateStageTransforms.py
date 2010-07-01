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
                (self.trans,self.rot) = self.tf_listener.lookupTransform("Plate", "Stage", rospy.Time.now())
                self.transforms_initialized = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # rospy.logwarn("trans = %s" % (str(self.trans)))
        # rospy.logwarn("rot = %s" % (str(self.rot)))

        self.T = tf.transformations.translation_matrix(self.trans)
        self.R = tf.transformations.quaternion_matrix(self.rot)
        self.M = tf.transformations.concatenate_matrices(self.R, self.T)
        self.Minv = numpy.linalg.inv(self.M)
        # rospy.logwarn("T = \n%s" % (str(self.T)))
        # rospy.logwarn("R = \n%s" % (str(self.R)))
        Xsrc = [0,10]
        Ysrc = [0,10]
        Zsrc = [0,0]
        Hsrc = [1,1]
        plate_points = numpy.array([Xsrc,Ysrc,Zsrc,Hsrc])
        rospy.logwarn("M = \n%s" % (str(self.M)))
        rospy.logwarn("plate_points = \n%s" % (str(plate_points)))
        stage_points = numpy.dot(self.M,plate_points)
        rospy.logwarn("stage_points = \n%s" % (str(stage_points)))

        plate_point = PointStamped()
        plate_point.header.frame_id = "Plate"
        plate_point.point.x = 0
        plate_point.point.y = 0
        plate_point.point.z = 0
        stage_point = self.tf_listener.transformPoint("Stage","Plate")
        rospy.logwarn("plate_point1.x = \n%s" % (str(plate.point.point.x)))
        rospy.logwarn("plate_point1.y = \n%s" % (str(plate.point.point.y)))
        rospy.logwarn("stage_point1.x = \n%s" % (str(stage.point.point.x)))
        rospy.logwarn("stage_point1.y = \n%s" % (str(stage.point.point.y)))
        plate_point.point.x = 10
        plate_point.point.y = 10
        stage_point = self.tf_listener.transformPoint("Stage","Plate")
        rospy.logwarn("plate_point2.x = \n%s" % (str(plate.point.point.x)))
        rospy.logwarn("plate_point2.y = \n%s" % (str(plate.point.point.y)))
        rospy.logwarn("stage_point2.x = \n%s" % (str(stage.point.point.x)))
        rospy.logwarn("stage_point2.y = \n%s" % (str(stage.point.point.y)))

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
