#!/usr/bin/env python
import roslib
roslib.load_manifest('stage_tf')
import rospy

import tf
# from geometry_msgs.msg import PoseStamped,PointStamped
from stage.msg import StateStamped
# from plate_tf.srv import *

class PoseTFConversion:
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.magnet_stage_state_sub = rospy.Subscriber('MagnetStageState',StateStamped,self.handle_magnet_stage_state)

    def handle_magnet_stage_state(self,msg):
        self.tf_broadcaster.sendTransform((msg.x, msg.y, 0),
                                          tf.transformations.quaternion_from_euler(0, 0, 0),
                                          rospy.Time.now(),
                                          "Magnet",
                                          "Stage")

if __name__ == '__main__':
    rospy.init_node('magnet_tf_broadcaster')
    ptc = PoseTFConversion()
    while not rospy.is_shutdown():
        rospy.spin()
