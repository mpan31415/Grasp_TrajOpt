#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


######################### Note: fetch_moveit_config move_group.launch must be running ############################
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.


if __name__ == '__main__':

    rospy.init_node("entire_robot_control")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane
    planning_scene = PlanningSceneInterface("base_link")


    # # TF joint names
    # joint_names = ["torso_lift_joint", "shoulder_pan_joint",
    #                "shoulder_lift_joint", "upperarm_roll_joint",
    #                "elbow_flex_joint", "forearm_roll_joint",
    #                "wrist_flex_joint", "wrist_roll_joint"]
    # # Lists of joint angles in the same order as in joint_names
    # joint_goal = [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]

    # # Plans the joints in joint_names to angles in pose
    # move_group.moveToJointPosition(joint_names, joint_goal, wait=False)

    # # Since we passed in wait=False above we need to wait here
    # move_group.get_move_action().wait_for_result()
    # result = move_group.get_move_action().get_result()


    #################### Planning to a pose goal in 3D world frame ######################
    pose_goal = PoseStamped()

    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.8
    pose_goal.pose.position.z = 1.0

    move_group.moveToPose(pose_goal, wait=False)

    # Since we passed in wait=False above we need to wait here
    move_group.get_move_action().wait_for_result()
    result = move_group.get_move_action().get_result()



    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Disco!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                        move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")


    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()