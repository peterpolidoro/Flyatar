#! /usr/bin/env python
import roslib; roslib.load_manifest('flyatar_action_client')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the stage_action_server action, including the
# goal message and the result message.
import stage_action_server.msg

def flyatar_action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('StageActionServer', stage_action_server.msg.UpdateStagePositionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = stage_action_server.msg.UpdateStagePositionGoal()

    # Home
    goal.x_position = []
    goal.y_position = []
    goal.velocity_magnitude = []

    # Sends the goal to the action server.
    rospy.logwarn("Sending goal...")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    rospy.logwarn("result = %s" % (str(result)))

    num_trials = 3
    for trial in range(num_trials):
        if trial == 0:
            x_pos_list = [100,150,150,100,100]
            y_pos_list = [100,100,150,150,100]
        else:
            x_pos_list.reverse()
            y_pos_list.reverse()
        goal.x_position = x_pos_list
        goal.y_position = y_pos_list
        goal.velocity_magnitude = [50]

        # Sends the goal to the action server.
        rospy.logwarn("Sending goal...")
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        result = client.get_result()
        rospy.logwarn("result = %s" % (str(result)))

        goal.x_position = [125]
        goal.y_position = [125]
        goal.velocity_magnitude = [50]

        # Sends the goal to the action server.
        rospy.logwarn("Sending goal...")
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        result = client.get_result()
        rospy.logwarn("result = %s" % (str(result)))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('FlyatarActionClient')
        flyatar_action_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
