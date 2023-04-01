#!/usr/bin/env python

import sys
import math
import numpy as np

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

from to_euler import ToEulerAngles


class PublisherSubscriber:

    def __init__(self, publishTopicName, subscribeTopicName, target):
        
        self.target_index = target
        self.listener = rospy.Subscriber(subscribeTopicName, ModelStates, self.callback, queue_size=1, buff_size=2**24)
        self.talker = rospy.Publisher(publishTopicName, Twist, queue_size=10)


    def callback(self, data):

        robot_index = len(data.name) - 1
        target_index = self.target_index

        heading_thres = 3 # in degrees
        dist_thres = 1 # in meters
        angle_factor = 0.03
        vel_factor = 0.2

        # initialize empty message of type "Twist"
        msg = Twist()

        # current robot position (Note: z-position ignored as it's ZERO for both robot and target)
        rx = (data.pose)[robot_index].position.x
        ry = (data.pose)[robot_index].position.y
        ro = (data.pose)[robot_index].orientation # robot orientation specified in a Quaternion
        yaw = ToEulerAngles(ro.x, ro.y, ro.z, ro.w) # in degrees from positive x-axis

        # current target position
        tx = (data.pose)[target_index].position.x
        ty = (data.pose)[target_index].position.y

        # compute Euclidean distance and angle of target w.r.t robot in world frame
        dx = tx - rx
        dy = ty - ry
        dist = math.sqrt(dx**2 + dy**2)
        theta = math.degrees(np.arctan2(dy, dx)) # in degrees from positive x-axis

        rospy.loginfo("yaw: %.2f, theta: %.2f", yaw, theta)

        # first have to correct the robot heading
        if abs(yaw - theta) > heading_thres:
            
            wz = (theta - yaw) * angle_factor
            msg.angular.z = wz

            self.talker.publish(msg)
            rospy.loginfo("Setting robot to the correct heading with angular velocity %.2f ... ", wz)

        else:
            # robot is in the correct heading, move straight towards target
            # compute and set robot velocity
            vx = abs(dx) * vel_factor
            msg.linear.x = vx

            # display information in terminal (while the velocity is still larger than threshold)
            if dist > dist_thres:
                self.talker.publish(msg)      
                rospy.loginfo("Moving robot towards target with velocity %.2f ... ", vx)
            else:
                rospy.loginfo("The robot is close enough to the target, stopping movement!")
            



if __name__ == '__main__':
    
    target = int(sys.argv[1])
    
    rospy.init_node('publisher_subscriber', anonymous=True)
    PublisherSubscriber("/base_controller/command", "/gazebo/model_states", target)
    rospy.spin()


