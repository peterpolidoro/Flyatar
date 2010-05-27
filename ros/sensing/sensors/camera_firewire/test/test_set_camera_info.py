#!/usr/bin/env python
import roslib; roslib.load_manifest('camera1394')

import sys

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

service_name = 'camera/set_camera_info'

def test():
    rospy.wait_for_service(service_name)

    # build theoretical calibration for Unibrain Fire-i camera in
    # 640x480 mode
    cinfo = CameraInfo()
    cinfo.width = 640
    cinfo.height = 480
    cx = (cinfo.width - 1.0)/2.0
    cy = (cinfo.height - 1.0)/2.0
    fx = fy = 0.0043                    # Unibrain Fire-i focal length
    cinfo.K = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    cinfo.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    cinfo.P = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

    try:
        set_camera_info = rospy.ServiceProxy(service_name, SetCameraInfo)
        success = set_camera_info(cinfo)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('set_camera_info test completed')
