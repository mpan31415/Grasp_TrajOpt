#!/usr/bin/env python

import sys
import math
import rospy
import time
import actionlib
import os
import rospkg

import tf
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_python import PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive

import moveit_commander

CLOSED = 0
OPEN = 0.1
MAX_EFFORT = 100 # seems to be percentage

# declare parent and child frames for ar marker
parent_frame = '/base_link'
shelf_frame = '/shelf'
box_frame_1 = '/cube_1'
box_frame_2 = '/cube_2'

shelf_name = "shelf"
object_name = "box_1"
object_name_2 = "box_2"
end_effector = "gripper_link"
planner_id = "RRTstarkConfigDefault"
planning_time = 20
vel_factor = 0.1

got_shelf = False
got_box_1 = False
got_box_2 = False


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


def scan():
    global got_shelf
    global got_box_1
    global got_box_2

    box_1_pos = [0, 0, 0]
    box_2_pos = [0, 0, 0]
    #######################################################################################
    ############################## CAN WE GET THE SHELF POSITION? #########################
    try:
        (trans,rot_quart) = listener.lookupTransform(parent_frame, shelf_frame, rospy.Time(0))

        # convert from quaternion to euler angles
        rot_euler = list(euler_from_quaternion(rot_quart))
        # print(rot_euler)
        # rot_euler[0] = 0
        # rot_euler[1] = 0
        # rot_quart = quaternion_from_euler(rot_euler[0],rot_euler[1],rot_euler[2])
        for rad in rot_euler:
            rad = rad2deg(rad)

        # compute angles for shelf relative to alvar marker
        yaw = rot_euler[2]
        theta = yaw + alpha

        # print("x: "+str(trans[0]))
        # print("y: "+str(trans[1]))
        # print("z: "+str(trans[2]))
        # print("yaw: "+str(yaw))

        # calculate shelf translation in robot's base_link frame
        shelf_x = trans[0]
        shelf_y = trans[1]
        shelf_z = trans[2] # assuming shelf is level with the fetch's base link

        # calculate shelf rotation in robot's base_link frame
        right_angle = quaternion_from_euler(1.5707, 0, 0)
        result_quart = quaternion_multiply(rot_quart, right_angle)
        result_quart_msg = Quaternion(result_quart[0],result_quart[1],result_quart[2],result_quart[3])

        # generate shelf pose and add to move_it planning scene
        shelf_pose = Pose(Point(shelf_x, shelf_y, shelf_z), result_quart_msg)
        if (os.path.exists(path+"/shelf.stl")):
            print("Found File! Adding mesh...")
            planning_scene.addMesh(shelf_name, shelf_pose, path+"/shelf.stl")
            got_shelf = True
        else:
            print("Did not find file.")


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("failed to get pose of alvar marker 4")
        # continue

    #################################################################################################
    ############################## CAN WE GET THE RUBIX CUBE POSITION? ##############################
    try:
        (trans,rot_quart) = listener.lookupTransform(parent_frame, box_frame_1, rospy.Time(0))

        # convert from quaternion to euler angles
        rot_euler = list(euler_from_quaternion(rot_quart))
        # print(rot_euler)
        # rot_euler[0] = 0
        # rot_euler[1] = 0
        # rot_quart = quaternion_from_euler(rot_euler[0],rot_euler[1],rot_euler[2])
        for rad in rot_euler:
            rad = rad2deg(rad)

        # compute angles for shelf relative to alvar marker
        yaw = rot_euler[2]
        theta = yaw + alpha

        print("box 1")
        print("x: "+str(trans[0]))
        print("y: "+str(trans[1]))
        print("z: "+str(trans[2]))
        print("yaw: "+str(yaw))

        # calculate shelf translation in robot's base_link frame
        box_1_pos = trans

        # generate shelf pose and add to move_it planning scene
        s = SolidPrimitive()
        s.dimensions = [0.055, 0.055, 0.08]
        s.type = s.BOX

        # planning_scene.addBox(object_name, 0.055, 0.055, 0.08, trans[0], trans[1], trans[2])
        right_angle = quaternion_from_euler(1.5707, 0, 0)
        result_quart = quaternion_multiply(rot_quart, right_angle)
        result_quart_msg = Quaternion(result_quart[0],result_quart[1],result_quart[2],result_quart[3])
        pose = Pose(Point(trans[0], trans[1], trans[2]), result_quart_msg)
        planning_scene.addSolidPrimitive(object_name, s, pose)

        got_box_1 = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("failed to get pose of alvar marker 7")
        box_1_pos = None
        # continue

    ##############################################################################################
    ############################## CAN WE GET THE HONEY'S POSITION? ##############################
    try:
        (trans,rot_quart) = listener.lookupTransform(parent_frame, box_frame_2, rospy.Time(0))

        # convert from quaternion to euler angles
        rot_euler = list(euler_from_quaternion(rot_quart))
        # print(rot_euler)
        # rot_euler[0] = 0
        # rot_euler[1] = 0
        # rot_quart = quaternion_from_euler(rot_euler[0],rot_euler[1],rot_euler[2])
        for rad in rot_euler:
            rad = rad2deg(rad)

        # compute angles for shelf relative to alvar marker
        yaw = rot_euler[2]
        theta = yaw + alpha

        print("box 2")
        print("x: "+str(trans[0]))
        print("y: "+str(trans[1]))
        print("z: "+str(trans[2]))
        print("yaw: "+str(yaw))

        # calculate shelf translation in robot's base_link frame
        box_2_pos = trans

        # generate shelf pose and add to move_it planning scene
        # planning_scene.addBox(object_name_2, 0.066, 0.066, 0.066, trans[0], trans[1], trans[2])

        s = SolidPrimitive()
        s.dimensions = [0.066, 0.066, 0.066]
        s.type = s.BOX

        result_quart_msg = Quaternion(rot_quart[0],rot_quart[1],rot_quart[2],rot_quart[3])
        pose = Pose(Point(trans[0], trans[1], trans[2]), result_quart_msg)
        planning_scene.addSolidPrimitive(object_name_2, s, pose)

        got_box_2 = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("failed to get pose of alvar marker 1")
        box_2_pos = None
        # continue

    print("")

    return box_1_pos, box_2_pos


def sweep_and_scan():

    print("Scanning...")
    box_1_pos = [0, 0, 0]
    box_2_pos = [0, 0, 0]
    # initialize moveit_commander and a rospy node
    head_joint_positions_1 = [0.5,0.3]
    head_joint_positions_2 = [0.0, 0.3]
    head_joint_positions_3 = [-0.5, 0.3]

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    point_head(head_client,head_joint_positions_1)
    box_1, box_2 = scan()
    if (box_1 is not None):
        box_1_pos = box_1
    if (box_2 is not None):
        box_2_pos = box_2
    point_head(head_client,head_joint_positions_2)
    box_1, box_2 = scan()
    if (box_1 is not None):
        box_1_pos = box_1
    if (box_2 is not None):
        box_2_pos = box_2
    point_head(head_client,head_joint_positions_3)
    box_1, box_2 = scan()
    if (box_1 is not None):
        box_1_pos = box_1
    if (box_2 is not None):
        box_2_pos = box_2

    return box_1_pos, box_2_pos


def move_box_to_basket(object_name, box_x, box_y, box_z):
    # initialize move_group and update to the desired parameters
    move_group = moveit_commander.MoveGroupCommander("arm")
    
    move_group.set_max_velocity_scaling_factor(vel_factor)
    move_group.set_planning_time(planning_time)

    # move_group.set_planner_id(planner_id)
    print("The planner id is %s yeah" % move_group.get_planner_id())

    pose_goal = Pose()

    # first goal is the object
    #pose_goal.orientation.w = 1.0
    # pose_goal.position.x = box_x
    # pose_goal.position.y = box_y
    # pose_goal.position.z = box_z + 0.02
    pose_goal.position.x = box_x
    pose_goal.position.y = box_y
    pose_goal.position.z = box_z

    quart_angle = quaternion_from_euler(0.0, 0.0, 0.0)

    print(pose_goal.position)
    print(quart_angle)

    pose_goal.orientation.x = quart_angle[0]
    pose_goal.orientation.y = quart_angle[1]
    pose_goal.orientation.z = quart_angle[2]
    pose_goal.orientation.w = quart_angle[3]

    # set goal 1 to be the object
    move_group.set_pose_target(pose_goal, end_effector)
    print("Goal: [%f, %f, %f]" % (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z))

    # get the planner to plan and execute it, then clear pose targets (good practice)
    plan = move_group.go(wait=True)
    move_group.clear_pose_targets()

    ######################### ensure that there is no residual movement #########################
    move_group.stop()
    # time.sleep(1)

    # move_group has finished its job, now need to grasp the object
    print("Arrived at Goal, Gripping...")
    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    client.wait_for_server()

    # define goal 2 (the grasp goal)
    goal = GripperCommandGoal()
    goal.command.position = CLOSED
    goal.command.max_effort = MAX_EFFORT

    # send goal to action server and wait for feedback
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    rospy.loginfo("waiting for action result...")

    move_group_2 = moveit_commander.MoveGroupCommander("arm")
    
    move_group_2.set_max_velocity_scaling_factor(vel_factor)
    move_group_2.set_planning_time(planning_time)

    # move_group_2.set_planner_id(planner_id)

    # now has object in grasp, need to attach it to become part of robot arm, and then plan to the basket position
    move_group_2.attach_object(object_name, end_effector, ["l_gripper_finger_link", "r_gripper_finger_link"])

    # define basket position w.r.t the shelf
    basket_x = 0.0
    basket_y = 0.6
    basket_z = 0.4

    # define goal 3 for the arm to move the object to
    basket_goal = Pose()
    basket_goal.position.x = basket_x
    basket_goal.position.y = basket_y
    basket_goal.position.z = basket_z

    quart_angle = quaternion_from_euler(0.0, 1.57, 0.0)
    #print(quart_angle)

    basket_goal.orientation.x = quart_angle[0]
    basket_goal.orientation.y = quart_angle[1]
    basket_goal.orientation.z = quart_angle[2]
    basket_goal.orientation.w = quart_angle[3]

    # set goal 3 to be the basket position
    move_group_2.set_pose_target(basket_goal, end_effector)
    print("Goal: [%f, %f, %f]" % (basket_goal.position.x, basket_goal.position.y, basket_goal.position.z))

    # get the planner to plan and execute it, then clear pose targets (good practice)
    plan = move_group_2.go(wait=True)
    move_group_2.clear_pose_targets()

    ######################### ensure that there is no residual movement #########################
    move_group_2.stop()


    # move_group has finished its job, now need to release the object
    print("Arrived at Goal, Releasing...")
    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    client.wait_for_server()

    # define goal 2 (the grasp goal)
    goal = GripperCommandGoal()
    goal.command.position = OPEN
    goal.command.max_effort = MAX_EFFORT

    # send goal to action server and wait for feedback
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    rospy.loginfo("waiting for action result...")

    planning_scene.removeAttachedObject(object_name)
    planning_scene.removeCollisionObject(object_name)


def arm_tuck():
    move_group = moveit_commander.MoveGroupCommander("arm")
        
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_planning_time(20)

    arm_tuck_pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    move_group.go(arm_tuck_pose, wait=True)
    move_group.clear_pose_targets()

    move_group.stop()



if __name__ == '__main__':

    # initializes node
    rospy.init_node('alvar_watcher_runner', anonymous=True)

    rospack = rospkg.RosPack()
    path = rospack.get_path("loading_test")
    print(path)

    # define some useful constants
    x_4 = 0.275 + 0.02
    y_4 = 0.595 + 0.02
    alpha = math.atan(x_4 / y_4)
    hypo = math.sqrt(x_4**2 + y_4**2)

    # function that converts radians to degrees
    def rad2deg(rad):
        return rad / math.pi * 180

    # instantiate objects
    planning_scene = PlanningSceneInterface("base_link")
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    planning_scene.clear()

    # while not rospy.is_shutdown():
    #     shelf_name = "shelf"
    #     object_name = "box_1"
    #     object_name_2 = "box_2"
    #     end_effector = "gripper_link"
    #     planner_id = "RRTstar"
    #     scan()
    
    # loop while ros is running
    while not rospy.is_shutdown():

        [box_1_pos, box_2_pos] = sweep_and_scan()

        ###############################################################################################
        ################################ NOW WE HAVE BOTH, GO FOR IT! #################################
        # continue
        if (got_box_1 and got_box_2 and got_shelf):
            print("Got both Box and Shelf, planning and running...")
            # time.sleep(10)

            move_box_to_basket(object_name_2, box_2_pos[0]- 0.03, box_2_pos[1], box_2_pos[2] + 0.03)
            move_box_to_basket(object_name, box_1_pos[0], box_1_pos[1], box_1_pos[2] + 0.03)

            arm_tuck()
            break

    # rospy.spin()


    
