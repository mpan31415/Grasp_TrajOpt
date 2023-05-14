#!/usr/bin/env python

import rospy
import math
import time
import actionlib
import os
import csv
from datetime import datetime
import random
import sys

import tf
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_python import PlanningSceneInterface
# from moveit_commander import PlanningSceneInterface

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from michael_msgs.msg import OmplRequest, OmplResponse
from michael_msgs.srv import GetOmplResults

import moveit_commander

from moveit_msgs.msg import RobotState, CollisionObject
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


init_poses_set = [
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

# INIT_POSE_LIST = init_poses_set[2]
# INIT_POSE_LIST = [1.32, 1.4, -0.2, 1.72, 0, 1.66, 0]
INIT_POSE_LIST = [-1, -0.3, 1.4816, 0.9, -3, -0.462, 2.8155]


################ GRIPPER CONSTANTS ###############
CLOSED = 0
OPEN = 0.1
MAX_EFFORT = 100 # seems to be percentage

##################### OBJECT SIZES AND OTHER CONSTANTS ###################
BOX_HEIGHT = 0.25
BOX_WIDTH = 0.06

RADIUS = 0.03
HEIGHT = 0.1
OFFSET = HEIGHT / 2

# the following are in cm
OBJ_WIDTH = RADIUS * 2 * 100
OBJ_HEIGHT = HEIGHT * 100

S_LOW = 0.3 - 0.03*(OBJ_HEIGHT - 6)
S_HIGH = 0.7 + 0.03*(OBJ_WIDTH - 6)
THETA_LOW = 0
THETA_HIGH = 1.2    # (in radians)

VEL_COEFF = 1
CONST_X = -0.18



class RRTCost:

    def __init__(self, velocity, acceleration, jerk, total):
        self.velocity = velocity
        self.accelertion = acceleration
        self.jerk = jerk
        self.total = total


class ExperimentData:

    def __init__(self, length, cost, solve_time, success_rate ):
        self.data = {
            'Trajectory Length': length,
            'Velocity Cost': cost.velocity,
            'Acceleration Cost': cost.accelertion,
            'Jerk Cost': cost.jerk,
            'Total Cost': cost.total,
            'Solve Time': solve_time,
            'Success Rate': success_rate
        }

    def save_data_to_csv(self, filename):
        # Check if the file already exists
        file_exists = os.path.isfile(filename)
        
        # Initialize the trial ID to 1 if the file doesn't exist
        trial_id = 1
        
        # If the file exists, read the last trial ID and increment it
        if file_exists:
            with open(filename, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    trial_id = int(row['Trial ID']) + 1
        
        self.data['Trial ID'] = trial_id
        self.trial_id = trial_id

        # Open the file in append mode
        with open(filename, 'a') as csvfile:
            fieldnames = ['Trial ID', 'Date', 'Trajectory Length', 'Velocity Cost', 
                        'Acceleration Cost', 'Jerk Cost', 'Total Cost', 'Solve Time', 'Success Rate']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            # Write the header row if the file doesn't exist
            if not file_exists:
                writer.writeheader()
            
            # Add the current date to the data dictionary
            self.data['Date'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # Write the data to the file
            writer.writerow(self.data)


################################# GRASP PARAMETERIZATION USING "BELL-CURVE" WEIGHTS ################################
def hand_param2(s, theta):

    var = 0.2
    width = OBJ_WIDTH
    height = OBJ_HEIGHT
    w5 = (1 / (2*var*math.pi)) * math.exp(-1 * (theta - math.pi/4)**2 / var) if (theta > 0 and theta < math.pi/2) else 0
    w4 = (1 - w5) if (theta >= 0 and theta < math.pi/4) else 0
    w6 = (1 - w5) if (theta > math.pi/4 and theta <= math.pi/2) else 0

    x = -width/2 + (width-1) * w4 + (width+0.5) * w5 * (math.cos(math.pi/2 * s))**(1/(width-4)) + width * w6 * (1 - s)
    y = 1 + height * s * w4 + (height+0.5) * w5 * (math.sin(math.pi/2 * s))**(1/(height-4)) + (height-1) * w6

    return (x/100, y/100)


def get_diff(prev, next):
    diff_list = []
    sum = 0.0
    for i in range(len(prev)):
        diff_list.append(next[i] - prev[i])
        # sum += abs(next[i] - prev[i])
        sum += (next[i] - prev[i])**2
    return (sum, diff_list)



def get_costs(trajectory, ratio=1, printing=False):
        
    scaling = ratio

    if printing:
        
        print("=" * 100)
        print("Calculating trajectory length by doing a summation across differences in position between adjacent waypoints!")
        print("=" * 100)

    if trajectory is not None:
        # print("Found trajectory!")
        points = trajectory.points
        length = 0.0
        vel_cost = 0.0
        accel_cost = 0.0
        jerk_cost = 0.0
        pos_diffs = []
        vel_diffs = []
        accel_diffs = []
        ###########################
        # get pose differences
        for i in range(0, len(points) - 1):
            prev_point = points[i]
            next_point = points[i+1]
            (pos_diff, diff_list) = get_diff(prev_point.positions, next_point.positions)
            vel_cost += pos_diff * VEL_COEFF
            length += pos_diff
            pos_diffs.append(diff_list)
        length = length*scaling
        vel_cost = vel_cost*scaling
        if printing:
            print("The joint-space length of the trajectory is %.3f " % length)
            print("=" * 100)
            print("The velocity cost of the trajectory is %.3f " % vel_cost)
            print("=" * 100)
        ###########################
        # get vel differences
        for i in range(0, len(pos_diffs) - 1):
            prev_vel = pos_diffs[i]
            next_vel = pos_diffs[i+1]
            (vel_diff, diff_list) = get_diff(prev_vel, next_vel)
            accel_cost += vel_diff
            vel_diffs.append(diff_list)
        accel_cost = accel_cost*scaling
        if printing:
            print("The acceleration cost of the trajectory is %.3f " % accel_cost)
            print("=" * 100)
        ###########################
        # get accel differences
        for i in range(0, len(vel_diffs) - 1):
            prev_accel = vel_diffs[i]
            next_accel = vel_diffs[i+1]
            (accel_diff, diff_list) = get_diff(prev_accel, next_accel)
            jerk_cost += accel_diff
            accel_diffs.append(diff_list)
        jerk_cost = jerk_cost*scaling
        if printing:
            print("The jerk cost of the trajectory is %.3f " % jerk_cost)
            print("=" * 100)
        total_cost = vel_cost + accel_cost + jerk_cost
        if printing:
            print("The trajectory length is %.3f radians, and the total cost is %.3f !" % (total_cost, length))
        return (length, vel_cost, accel_cost, jerk_cost, total_cost)
    else:
        print("The input trajectory is empty!")



################################ CLOSE / OPEN GRIPPERS ###############################
def move_hand(type):

    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    client.wait_for_server()

    # define goal
    goal = GripperCommandGoal()
    goal.command.position = type
    goal.command.max_effort = MAX_EFFORT

    # send goal to action server and wait for feedback
    print("Moving gripper ... ")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))




class GazeboPlanner:

    def __init__(self):
        
        ################ OBJECT NAMES #################
        self.bench_name = "bench"
        self.beer1_name = "beer1"
        self.beer2_name = "beer2"
        self.beer3_name = "beer3"
        self.beer4_name = "beer4"
        self.beer5_name = "beer5"
        self.fetch_name = "fetch"

        self.end_effector = "gripper_link"
        self.planner_id = "RRTstarkConfigDefault"
        self.planning_time = 5
        self.vel_factor = 0.1
        
        self.logging = False

        self.planning_scene = PlanningSceneInterface("base_link")

        # self.bench_dir = "../../models/shelf_parts/shelf_2.STL"
        self.bench_dir = os.path.expanduser('~') + "/MonashRA/moveit_ws/src/models/shelf_parts/shelf_2.STL"

        self.got_scene = False

        self.bench_pose = None
        self.beer1_pose = None
        self.beer2_pose = None
        self.beer3_pose = None
        self.beer4_pose = None
        self.beer5_pose = None
        self.num_beers = 0

        self.grasp_pose = None

        self.trajectories = {}
        self.response = None
        

        self.obj_index = 3
        self.iterations = 200
        self.saving_data = True


    ########################## LISTENER FUNCTION ###################### 
    def run(self):

        rospy.init_node('gazebo_scene_getter', anonymous=True)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=1, buff_size=2**24)

        rospy.spin()

    
    ########################## CALLBACK FUNCTION ###################### 
    def callback(self, data):

        if not self.got_scene:

            collision_objects = self.planning_scene.getKnownCollisionObjects()
            # collision_objects = self.planning_scene.get_known_object_names()
            if self.logging:
                rospy.loginfo(" -> Collision objects in current Gazebo scene: %s " % collision_objects)

            # self.planning_scene.clear()

            list_len = len(data.name) # including "world_plane"

            robot_pos = [0, 0, 0]
            robot_quart = [0, 0, 0, 0]
            robot_euler = [0,0,0]

            for i in range(list_len):
                if "fetch" in (data.name)[i]:
                    robot_pos = [(data.pose)[i].position.x,(data.pose)[i].position.y,(data.pose)[i].position.z]
                    robot_quart = [(data.pose)[i].orientation.x,(data.pose)[i].orientation.y,(data.pose)[i].orientation.z,(data.pose)[i].orientation.w]
                    robot_euler = euler_from_quaternion(robot_quart)
                    if self.logging:
                        rospy.loginfo("Robot Position: "+str(robot_pos))
                        rospy.loginfo("Robot Euler: "+str(robot_euler))
            
            robot_x = robot_pos[0]
            robot_y = robot_pos[1]
            robot_yaw = robot_euler[2]


            for index in range(list_len):

                if self.logging:
                    rospy.loginfo(" -> The %s has position (%.3f, %.3f, %.3f)", 
                        (data.name)[index], (data.pose)[index].position.x, (data.pose)[index].position.y, (data.pose)[index].position.z)

                if self.beer1_name in (data.name)[index]:
                    self.num_beers += 1
                    beer_world_x = (data.pose)[index].position.x
                    beer_world_y = (data.pose)[index].position.y

                    beer_rel_x = (beer_world_x - robot_x) * math.cos(-robot_yaw) - (beer_world_y - robot_y) * math.sin(-robot_yaw)
                    beer_rel_y = (beer_world_x - robot_x) * math.sin(-robot_yaw) + (beer_world_y - robot_y) * math.cos(-robot_yaw)

                    beer_quart = (data.pose)[index].orientation
                    self.beer1_pose = Pose(Point(beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET), beer_quart)

                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    # self.planning_scene.addCylinder((data.name)[index], HEIGHT, RADIUS, beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET)
                    # self.planning_scene.add_box((data.name)[index], beer_pose)
                    self.planning_scene.addBox((data.name)[index], BOX_HEIGHT, BOX_WIDTH, BOX_HEIGHT, 
                                               beer_rel_x, beer_rel_y, (data.pose)[index].position.z + BOX_HEIGHT/2)
                
                if self.beer2_name in (data.name)[index]:
                    self.num_beers += 1
                    beer_world_x = (data.pose)[index].position.x
                    beer_world_y = (data.pose)[index].position.y

                    beer_rel_x = (beer_world_x - robot_x) * math.cos(-robot_yaw) - (beer_world_y - robot_y) * math.sin(-robot_yaw)
                    beer_rel_y = (beer_world_x - robot_x) * math.sin(-robot_yaw) + (beer_world_y - robot_y) * math.cos(-robot_yaw)

                    beer_quart = (data.pose)[index].orientation
                    self.beer2_pose = Pose(Point(beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET), beer_quart)

                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    # self.planning_scene.addCylinder((data.name)[index], HEIGHT, RADIUS, beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET)
                    self.planning_scene.addBox((data.name)[index], BOX_HEIGHT, BOX_WIDTH, BOX_HEIGHT, 
                                               beer_rel_x, beer_rel_y, (data.pose)[index].position.z + BOX_HEIGHT/2)
                
                if self.beer3_name in (data.name)[index]:
                    self.num_beers += 1
                    beer_world_x = (data.pose)[index].position.x
                    beer_world_y = (data.pose)[index].position.y

                    beer_rel_x = (beer_world_x - robot_x) * math.cos(-robot_yaw) - (beer_world_y - robot_y) * math.sin(-robot_yaw)
                    beer_rel_y = (beer_world_x - robot_x) * math.sin(-robot_yaw) + (beer_world_y - robot_y) * math.cos(-robot_yaw)

                    beer_quart = (data.pose)[index].orientation
                    self.beer3_pose = Pose(Point(beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET), beer_quart)

                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    # self.planning_scene.addCylinder((data.name)[index], HEIGHT, RADIUS, beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET)
                    self.planning_scene.addBox((data.name)[index], BOX_HEIGHT, BOX_WIDTH, BOX_HEIGHT, 
                                               beer_rel_x, beer_rel_y, (data.pose)[index].position.z + BOX_HEIGHT/2)

                if self.beer4_name in (data.name)[index]:
                    self.num_beers += 1
                    beer_world_x = (data.pose)[index].position.x
                    beer_world_y = (data.pose)[index].position.y

                    beer_rel_x = (beer_world_x - robot_x) * math.cos(-robot_yaw) - (beer_world_y - robot_y) * math.sin(-robot_yaw)
                    beer_rel_y = (beer_world_x - robot_x) * math.sin(-robot_yaw) + (beer_world_y - robot_y) * math.cos(-robot_yaw)

                    beer_quart = (data.pose)[index].orientation
                    self.beer4_pose = Pose(Point(beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET), beer_quart)

                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    self.planning_scene.addCylinder((data.name)[index], HEIGHT, RADIUS, beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET)

                if self.beer5_name in (data.name)[index]:
                    self.num_beers += 1
                    beer_world_x = (data.pose)[index].position.x
                    beer_world_y = (data.pose)[index].position.y

                    beer_rel_x = (beer_world_x - robot_x) * math.cos(-robot_yaw) - (beer_world_y - robot_y) * math.sin(-robot_yaw)
                    beer_rel_y = (beer_world_x - robot_x) * math.sin(-robot_yaw) + (beer_world_y - robot_y) * math.cos(-robot_yaw)

                    beer_quart = (data.pose)[index].orientation
                    self.beer5_pose = Pose(Point(beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET), beer_quart)

                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    self.planning_scene.addCylinder((data.name)[index], HEIGHT, RADIUS, beer_rel_x, beer_rel_y, (data.pose)[index].position.z + OFFSET)

                elif self.bench_name in (data.name)[index]:
                    # self.planning_scene.removeCollisionObject((data.name)[index])
                    shelf_quart = [(data.pose)[index].orientation.x,(data.pose)[index].orientation.y,(data.pose)[index].orientation.z,(data.pose)[index].orientation.w]

                    shelf_world_x = (data.pose)[index].position.x
                    shelf_world_y = (data.pose)[index].position.y

                    shelf_rel_x = (shelf_world_x - robot_x) * math.cos(-robot_yaw) - (shelf_world_y - robot_y) * math.sin(-robot_yaw)
                    shelf_rel_y = (shelf_world_x - robot_x) * math.sin(-robot_yaw) + (shelf_world_y - robot_y) * math.cos(-robot_yaw)

                    y_neg90 = quaternion_from_euler(0, -1.5708, 0)
                    x_neg90 = quaternion_from_euler(-1.5708, 0, 0)
                    
                    result_quart = quaternion_multiply(shelf_quart, quaternion_from_euler(0, 0, -robot_yaw))
                    result_quart = quaternion_multiply(result_quart, x_neg90)
                    result_quart_msg = Quaternion(result_quart[0],result_quart[1],result_quart[2],result_quart[3])

                    shelf_pose = Pose(Point(shelf_rel_x,shelf_rel_y,(data.pose)[index].position.z),
                                result_quart_msg)
                    self.planning_scene.addMesh((data.name)[index], shelf_pose, self.bench_dir)
                    # self.planning_scene.add_mesh((data.name)[index], shelf_pose, "../models/ShelfMini.stl")

            self.got_scene = True

        else:

            # object_choice = input("Enter the beer number to go for [1 - 3]: ")
            # object_choice = 3
            # dx = input("Enter dx: ")
            # dz = input("Enter dz: ")
            # theta = input("Enter theta: ")
            # yaw = input("Enter yaw: ")
            # plan(int(object_choice), float(dx), float(dz), float(theta), float(yaw))
            s = random.uniform(S_LOW, S_HIGH)
            theta = random.uniform(THETA_LOW, THETA_HIGH)
            s = 0.07
            theta = 0
            yaw = 0.0
            if self.logging:
                rospy.loginfo("The input parameters are (%.3f, %.3f)" % (s, theta))

            (dx, dz) = hand_param2(s, theta)

            # self.plan(int(object_choice), dx, dz, theta, yaw)
            # plan(int(object_choice), 0.02, 0.07, 0, 0)

            tic = time.time()

            for i in range(self.iterations):

                rospy.loginfo("Running iteration #%d out of %d "% (int(i+1), self.iterations))
                self.plan(self.obj_index, dx, dz, theta, yaw)

            toc = time.time()
            elapsed = toc - tic
            rospy.logerr("planning all trajectories took %.5f seconds!\n" % elapsed)

            rospy.signal_shutdown("goodbye")

            # trajectory_pub = rospy.Publisher('rrt_trajectories', OmplResponse, queue_size=10, latch=True)
            # rate = rospy.Rate(0.5) # 0.5 Hz

            # while not rospy.is_shutdown():

            #     if self.response is None:
            #         min_cost = sys.maxsize
            #         best_object = 0
            #         for obj_key in self.trajectories:
            #             info = self.trajectories[obj_key]
            #             if info[1] > 0 and info[1] < min_cost:
            #                 min_cost = info[1]
            #                 best_object = obj_key
            #         rospy.logerr("The best object is #%d, with a trajectory cost of %.3f" % (best_object, min_cost))
            #         best_trajectory = self.trajectories[best_object][0]
            #         response = OmplResponse()
            #         response.object_index = best_object
            #         response.init_pose = best_trajectory.points[0]
            #         response.trajectory = best_trajectory
            #         response.num_waypoints = len(best_trajectory.points)
            #         response.cost = min_cost
            #         response.planning_time = elapsed

            #         # Assign response class variable
            #         self.response = response
                
            #     else:
            #         rospy.loginfo("Now have self.response, publishing message now!\n")
            #         # rospy.loginfo("%s" % self.response)
            #         trajectory_pub.publish(self.response)
            #         rate.sleep()
            #         rospy.signal_shutdown("goodbye")


    ################################# ARM TUCK FUNCTION #################################
    def plan(self, obj_index, dx, dz, theta, yaw):
        
        tic = time.time()

        move_group = moveit_commander.MoveGroupCommander("arm")
        move_group.set_max_velocity_scaling_factor(self.vel_factor)
        move_group.set_planning_time(self.planning_time)

        ################### SET START STATE OF ROBOT ###################
        joint_start_state = JointState()
        joint_start_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 
                                  'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
        joint_start_state.position = INIT_POSE_LIST + [OPEN/2, OPEN/2]
        robot_start_state = RobotState()
        robot_start_state.joint_state = joint_start_state
        move_group.set_start_state(robot_start_state)


        #################### PLAN TOWARDS THE CHOSEN BEER POSE ####################
        # rospy.loginfo("incoming request: beer object %d's pose: " % obj_index)
        if obj_index == 1:
            # print(self.beer1_pose)
            wanted = self.beer1_pose
        if obj_index == 2:
            # print(self.beer2_pose)
            wanted = self.beer2_pose
        if obj_index == 3:
            # print(self.beer3_pose)
            wanted = self.beer3_pose
        if obj_index == 4:
            # print(self.beer3_pose)
            wanted = self.beer4_pose
        if obj_index == 5:
            # print(self.beer3_pose)
            wanted = self.beer5_pose

        wanted_trans = Point(wanted.position.x, wanted.position.y, wanted.position.z)
        wanted_rot = Quaternion(wanted.orientation.x, wanted.orientation.y, wanted.orientation.z, wanted.orientation.w)
        wanted_pose = Pose(wanted_trans, wanted_rot)

        self.grasp_pose = wanted_pose

        self.grasp_pose.position.z += BOX_HEIGHT / 3
        self.grasp_pose.position.x += CONST_X - 0.11
        # self.grasp_pose.position.z += (OFFSET / 2)
        # self.grasp_pose.position.z += dz

        # dx_for_fetch = dx * math.cos(yaw)
        # dy_for_fetch = dx * math.sin(yaw)

        # self.grasp_pose.position.x -= dx_for_fetch
        # self.grasp_pose.position.y -= dy_for_fetch

        # grasp_euler = [0.0, theta, yaw]
        # grasp_quat = quaternion_from_euler(grasp_euler[0], grasp_euler[1], grasp_euler[2])

        extra_rot = quaternion_from_euler(0, 0, 0)
        q1 = Quaternion(extra_rot[0], extra_rot[1], extra_rot[2], extra_rot[3])
        grasp_quat = quaternion_multiply([q1.x, q1.y, q1.z, q1.w], [wanted_rot.x, wanted_rot.y, wanted_rot.z, wanted_rot.w])

        self.grasp_pose.orientation.x = grasp_quat[0]
        self.grasp_pose.orientation.y = grasp_quat[1]
        self.grasp_pose.orientation.z = grasp_quat[2]
        self.grasp_pose.orientation.w = grasp_quat[3]


        if self.logging:
            rospy.loginfo("The grasp pose position is \n%s " % self.grasp_pose.position)
            rospy.loginfo("The grasp pose orientation is \n%s " % self.grasp_pose.orientation)

        plan = move_group.plan(self.grasp_pose)

        moveit_traj = plan[1]
        joint_trajectory = moveit_traj.joint_trajectory
        waypoints = len(joint_trajectory.points)
        ratio = waypoints / 43

        toc = time.time()
        elapsed = toc - tic

        # choice = input("RRTConnect has finished! Would you like to calculate the trajectory length [1 for YES / 0 for NO]: ")
        choice = 1
        if int(choice) == 1:
            (length, vel_cost, accel_cost, jerk_cost, total_cost) = get_costs(joint_trajectory, ratio=1)
            print("+" * 100)
            print("The total cost of the trajectory is %.3f as calculated by the get_costs() function" % total_cost)
            print("The detailed cost information are %.3f, %.3f, %.3f, with %d waypoints!" % (vel_cost, accel_cost, jerk_cost, waypoints))
            print("+" * 100)
            print("\n")

        if joint_trajectory is not None:
            rospy.loginfo("Plan was successful!!! Cost = %.3f\n\n" % total_cost)
            if self.saving_data:
                data = ExperimentData(length, RRTCost(vel_cost, accel_cost, jerk_cost, total_cost), 
                                    elapsed, 100)
                # data.save_data_to_csv("../results/box_occluded.csv")
                # data.save_data_to_csv("../results/cylinder_arm_tuck.csv")
                # data.save_data_to_csv("../results/cylinder_arm_up.csv")
                data.save_data_to_csv("../results/box_shelf.csv")

        else:
            rospy.loginfo("Plan was unsuccessful!")
            
        # time.sleep(1)
        move_group.clear_pose_targets()
        move_group.stop()




########################## MAIN FUNCTION ###################### 
if __name__ == '__main__':
    
    planner = GazeboPlanner()

    planner.run()
    