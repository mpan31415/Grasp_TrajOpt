#!/usr/bin/env python

######################## This script can only plan for the workspace where the robot has a "fixed" interface with the ground ###################

######################## This script uses the "moveit_commander" class from the moveit package #######################

import sys
import rospy
import moveit_commander
import actionlib
import time

from math import pi
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)

def point_head(head_client, head_joint_positions):
    print(head_joint_positions)

    head_joint_names = ["head_pan_joint", "head_tilt_joint"]
    trajectory = JointTrajectory()
    trajectory.joint_names = head_joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = head_joint_positions
    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
    trajectory.points[0].time_from_start = rospy.Duration(5.0)

    head_goal_1 = FollowJointTrajectoryGoal()
    head_goal_1.trajectory = trajectory
    head_goal_1.goal_time_tolerance = rospy.Duration(0.0)

    head_client.send_goal(head_goal_1)
    head_client.wait_for_result(rospy.Duration.from_sec(10.0))
    rospy.loginfo("waiting for action result...")

def control():

    # initialize moveit_commander and a rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_head_test', anonymous=True)

    # instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    head_joint_names = ["head_pan_joint", "head_tilt_joint"]
    head_joint_positions_1 = [0.5,0.3]
    head_joint_positions_2 = [0.0, 0.3]
    head_joint_positions_3 = [-0.5, 0.3]

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    point_head(head_client,head_joint_positions_1)
    # time.sleep(3)
    point_head(head_client,head_joint_positions_2)
    # time.sleep(3)
    point_head(head_client,head_joint_positions_3)

if __name__== "__main__":
    try:
        control()
    except rospy.ROSInterruptException:
        pass
