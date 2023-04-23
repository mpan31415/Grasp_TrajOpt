#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryActionGoal


if __name__ == '__main__':

    rospy.init_node('move_to_goal_client')
    client = actionlib.SimpleActionClient('move_to_goal', FollowJointTrajectoryActionGoal)
    client.wait_for_server()

    goal = FollowJointTrajectoryActionGoal()
    
    # Fill in information about goal


    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    rospy.loginfo("waiting for action result...")

