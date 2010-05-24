#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('stage')
import rospy
import StageDevice
import threading
from stage.msg import Velocity, StateStamped

class StageUSB():
    def __init__(self):
        self.lock = threading.Lock()
        print "Opening XYFly stage device..."
        self.dev = StageDevice.StageDevice()
        self.dev.print_values()

    def update_velocity(self,x_velocity,y_velocity):
        self.dev.update_velocity(x_velocity,y_velocity)

    def return_state(self):
        x,y,theta,x_velocity,y_velocity,theta_velocity = self.dev.return_state()
        return x,y,theta,x_velocity,y_velocity,theta_velocity

    def close(self):
        self.dev.close()
        print "XYFly stage device closed."

class StageTalker(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.state_pub = rospy.Publisher("MagnetStageState",StateStamped)
        self.state_values = StateStamped()

    def talker(self):
        x,y,theta,x_velocity,y_velocity,theta_velocity = stage_usb.return_state()
        self.state_values.header.stamp = rospy.Time.now()
        self.state_values.x = x
        self.state_values.y = y
        self.state_values.theta = theta
        self.state_values.x_velocity = x_velocity
        self.state_values.y_velocity = y_velocity
        self.state_values.theta_velocity = theta_velocity
        self.state_pub.publish(self.state_values)

    def run(self):
        while not rospy.is_shutdown():
            stage_usb.lock.acquire()
            self.talker()
            stage_usb.lock.release()
            rospy.sleep(0.25)

class StageListener(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.vel_sub = rospy.Subscriber("stage/command_velocity", Velocity, self.velocity_callback)
        self.min_velocity = 1

    def velocity_callback(self,data):
        x_velocity = data.x_velocity
        y_velocity = data.y_velocity
        if abs(x_velocity) < self.min_velocity:
            x_velocity = 0
        if abs(y_velocity) < self.min_velocity:
            y_velocity = 0
        stage_usb.lock.acquire()
        stage_usb.update_velocity(x_velocity,y_velocity)
        stage_usb.lock.release()

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('StageCommunicator', anonymous=True)
    stage_usb = StageUSB()
    listener = StageListener()
    talker = StageTalker()
    listener.start()
    talker.start()

    while not rospy.is_shutdown():
        rospy.spin()

    listener.join()
    talker.join()
    stage_usb.close()
