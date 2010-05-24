#!/usr/bin/env python
import roslib
roslib.load_manifest('auto')
import rospy
import math
import tf
from stage.msg import Velocity, StateStamped

if __name__ == '__main__':
    rospy.init_node('auto_follower')

    self.tf_listener = tf.TransformListener()

    stage_vel = rospy.Publisher('stage/command_velocity', Velocity)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/Robot', '/Fly', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        turtle_vel.publish(turtlesim.msg.Velocity(linear, angular))

        rate.sleep()
