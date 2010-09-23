#! /usr/bin/env python
import roslib; roslib.load_manifest('flyatar_action_client')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the stage_action_server action, including the
# goal message and the result message.
import stage_action_server.msg

class FlyatarActionClient():
    def __init__(self):
        # Creates the SimpleActionClient, passing the type of the action
        # to the constructor.
        self.client = actionlib.SimpleActionClient('StageActionServer', stage_action_server.msg.UpdateStagePositionAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        self.goal = stage_action_server.msg.UpdateStagePositionGoal()

    def send_goal(self):
        # Sends the goal to the action server.
        self.client.send_goal(self.goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        # Prints out the result of executing the action
        self.result = self.client.get_result()
        rospy.logwarn("result = %s" % (str(self.result)))

    def home(self):
        self.goal.x_position = []
        self.goal.y_position = []
        self.goal.velocity_magnitude = []
        self.send_goal()

    def square_move(self,trial=0):
        if trial == 0:
            self.x_pos_list = [100,150,150,100,100]
            self.y_pos_list = [100,100,150,150,100]
        else:
            self.x_pos_list.reverse()
            self.y_pos_list.reverse()

        self.goal.x_position = self.x_pos_list
        self.goal.y_position = self.y_pos_list
        self.goal.velocity_magnitude = [50]
        self.send_goal()

    def center_move(self):
        self.goal.x_position = [125]
        self.goal.y_position = [125]
        self.goal.velocity_magnitude = [50]
        self.send_goal()

    def main(self):
        self.num_trials = 3
        for trial in range(self.num_trials):
            self.square_move(trial)
            self.center_move()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('FlyatarActionClient')
        fac = FlyatarActionClient()
        fac.main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
