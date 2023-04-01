#!/usr/bin/env python

######################## This script can only plan for the workspace where the robot has a "fixed" interface with the ground ###################

######################## This script uses the "moveit_commander" class from the moveit package #######################

import sys
import random
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
from moveit_python import PlanningSceneInterface
from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    RobotTrajectory,
    MoveItErrorCodes,
    RobotState
)

# planner_id = "RRTstarkConfigDefault"
planner_id = "RRTStar"

init_poses = [
    [0.6159923192225398, -0.9346678162103055, 0.4857694749341781, -1.4172700629835018, 1.7821330230280088, 1.678379277996719, -2.949236071566051],
    [1.08841624593772, 0.8943111501003149, -1.318615620490535, 0.36913892249297353, -2.7935966515994872, -2.1453702533617616, -3.102875142701485],
    [-0.8016202596880495, -0.706683514838107, 2.736419468386017, 0.9706119591570457, -2.9218787720247152, -0.7806565210595728, 2.7484821773834076],
    [-1.0405065991804003, 1.0239163685494566, 0.21350400205295594, 2.028911804120988, 2.3319455565612115, 0.2278766487166286, 0.9832641066110552],
    [-1.5755396254777907, -1.0223643829899376, 1.103605711047634, 2.0141902914349923, -2.5506333020350316, -1.023785656951368, -2.639608475545289],
    [0.4032597198933363, 1.3584234568455722, 2.565115813586912, 1.7218666878757065, 0.5283870010737761, 1.5986101763695482, -1.189407550455789],
    [-0.09162396746054302, -0.7324493979387918, 2.8612024526124626, 1.351041175460443, -1.6385629800394417, 0.6043026385456325, 0.4387621280894991],
    [1.5367209596291183, -0.919770578952972, 2.9108712234132748, 1.457555177662056, -1.4264319982868636, -1.121196367777884, -0.8592025121591038],
    [-0.6682099353577942, 1.3507470247009765, 1.803848625810029, 1.716726985250134, -0.8944314271733909, 2.017782930918038, 2.912011398357566],
    [-1.2987258392214773, -0.2471269344335889, -0.5921026249595451, -1.6182457547467202, -2.2031208749304176, 0.24256129246205083, -0.5048964119907375],
    [-0.05214215348213913, -1.1079038512723056, 2.7686662322103084, 0.5037717335480263, 2.5015926863136855, 2.062678933627904, -2.668266604768545],
    [0.6184936597052961, 0.10678567149722928, 1.9407147545365158, -2.167338645604439, 0.18503370858524182, -0.9436562323942781, 0.8708092266653509],
    [1.3532527870472522, -0.6924434997362552, 1.2149094075905005, -1.3010089238635265, 2.56441727166698, 1.4754088782146573, -2.7878272567210325],
    [0.8996744742058218, 0.024321491652633753, 0.7155160837969041, -2.2088666152413934, 1.2923617254475683, -2.006448098011315, 2.737488083229543],
    [-0.5750779178291558, -0.7645345548153856, 0.5978872327231057, -1.208316833560355, -2.677754798847056, 0.9641576480492948, -2.1974465185249827],
    [-1.3257611139547079, -1.1319224794884213, -1.9956122725070637, -1.3744452097839677, -2.296503321758442, 1.6780393130332234, -2.7993406282763145],
    [-0.2868458249017596, -0.9688920610074421, -2.566314609121558, 0.744293421263341, -1.1550443830462218, -0.2525449679419398, -1.0548668280318094],
    [-0.8227629323188215, 0.4455895520481279, 1.1567880965474595, 2.158372259961441, -1.027229381141212, 2.143807420358062, -2.2766244272541223],
    [0.6090702307105063, -0.48061131085222597, -0.7307305239851218, -1.966741734775249, -0.9702641849978542, 0.22482010524719964, -1.458217831734412],
    [0.2980681526754052, -1.1010277846176177, -2.14084930772112, 0.2509568266654387, 0.6098391212872385, -1.4503187366947534, -1.967923379176373]
]


def diff(way1, way2):
    prev_pos = way1.positions
    next_pos = way2.positions
    sum = 0.0
    for i in range(len(prev_pos)):
        sum += (prev_pos[i] - next_pos[i]) ** 2
    diff = math.sqrt(sum)
    return diff


def arm_tuck(index):

    rospy.init_node('tuck_arm_test', anonymous=True)

    print("Performing arm tuck!")

    move_group = moveit_commander.MoveGroupCommander("arm")
    # move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_planning_time(20)
    # move_group.set_planner_id(planner_id)

    # joint_state = JointState()
    # # joint_state.header = Header()
    # # joint_state.header.stamp = rospy.Time.now()
    # joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    # joint_state.position = init_poses[2]
    # moveit_robot_state = RobotState()
    # moveit_robot_state.joint_state = joint_state
    # move_group.set_start_state(moveit_robot_state)

    # move_group.set_planner_id(planner_id)

    # arm_tuck_pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    # plan = move_group.plan(init_poses[2])
    
    current_pose_list = init_poses[index]

    print("=" * 100)
    print("The current arm pose is: \n")
    print(current_pose_list)
    print("\n")
    print("=" * 100)

    move_group.go(current_pose_list, wait=True)
    move_group.clear_pose_targets()

    move_group.stop()


def control():

    # initialize moveit_commander and a rospy node
    # moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_test', anonymous=True)

    # instantiate a RobotCommander object
    # robot = moveit_commander.RobotCommander()

    # instantiate a PlanningSceneInterface object
    # scene = moveit_commander.PlanningSceneInterface()

    # instantiate a MoveGroupCommander object
    # planning_scene = PlanningSceneInterface("base_link")

    move_group = moveit_commander.MoveGroupCommander("arm")
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_planning_time(20)
    # move_group.set_planner_id(planner_id)

    # move_group.set_max_acceleration_scaling_factor(0.001)
    # move_group.set_planning_pipeline_id("ompl")

    # print(move_group.get_planning_pipeline_id)

    #################### planning to a joint goal ##################
    # joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = 0
    # joint_goal[2] = 0
    # joint_goal[3] = 0
    # joint_goal[4] = 0
    # joint_goal[5] = 0
    # joint_goal[6] = 0

    # move_group.set_joint_value_target(joint_goal)

    # # The go command can be called with joint values, poses, or without any
    # # parameters if you have already set the pose or joint target for the group
    # move_group.go(joint_goal, wait=True)


    #################### Planning to a pose goal in 3D world frame ######################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.orientation.x = -1.0
    pose_goal.position.x = 0.65
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.9

    move_group.set_pose_target(pose_goal)
    

    # get the planner to plan and execute it, then clear pose targets (good practice)
    # move_group.go(pose_goal, wait=True)
    # move_group.clear_pose_targets()


    ################# PLAN AND THEN EXECUTE SEPARATELY #################
    # trajectory = RobotTrajectory()

    plan = move_group.plan()
    # move_group.clear_pose_targets()

    # print("The plan has type %s yeah!" % type(plan))
    joint_trajectory = None

    if plan[0] == True:
        
        print("Trajectory is successfully found!")
        joint_trajectory = plan[1]
        # print("The joint trajectory has type %s yeah!" % type(joint_trajectory))
        
        waypoints = joint_trajectory.points

        # print("The header (%s) has type %s yeah!" % (header, type(header)))
        # print("The joint names are %s yeah!" % joint_names)
        num_waypoints = len(waypoints)
        print("The number of waypoints generated is %d" % num_waypoints)

        length = 0.0
        for i in range(0, num_waypoints - 1):
            # print("Currently prev is at the %d-th out of %d waypoints total\n" % (i+1, num_waypoints))
            length += diff(waypoints[i], waypoints[i+1])
        
        print("The total length of the trajectory is %.3f radians!" % length)


        # for i in range(len(waypoints)):
        #     print("The %d-th trajectory joint-space waypoint is %s \n" % (i, list(waypoints[i].positions)))
        
        # final_point = waypoints[len(waypoints)-1]
        # print("I think the final joint-space point is %s yeah!" % final_point)

        # final_positions = final_point.positions
        # time = final_point.time_from_start.secs

        # print(final_positions)
        # print("final positions tuple has type %s " % type(final_positions))
        # print("The execution of the path took %d seconds" % time)

    else:
        print("Trajectory not found :(")


    # move_group.set_joint_value_target(final_positions)

    # move_group.execute(plan, wait=True)
    
    ######################### ensure that there is no residual movement #########################
    move_group.clear_pose_targets()
    move_group.stop()




if __name__== "__main__":

    try:
        control()
    except rospy.ROSInterruptException:
        pass
