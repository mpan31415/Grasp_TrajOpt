#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


heading_thres = 3 # in degrees
dist_thres = 1 # in meters
angle_factor = 0.03
vel_factor = 0.2


def commander(pub, distance, angle, rate):

    while not rospy.is_shutdown():

        msg = Twist()

        # first have to correct the robot heading
        if abs(angle) > heading_thres:
            
            wz = -1 * angle * angle_factor
            msg.angular.z = wz

            pub.publish(msg)
            rospy.loginfo("Setting robot to the correct heading with angular velocity %.2f ... ", wz)
            rate.sleep()

        else:
            # robot is in the correct heading, move straight towards target
            # compute and set robot velocity
            vx = abs(distance) * vel_factor
            msg.linear.x = vx

            # display information in terminal (while the velocity is still larger than threshold)
            if distance > dist_thres:
                pub.publish(msg)      
                rospy.loginfo("Moving robot towards target with velocity %.2f ... ", vx)
                rate.sleep()
            else:
                rospy.loginfo("The robot is close enough to the target, stopping movement!")



if __name__ == '__main__':
    
    pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
    rospy.init_node('base_commander', anonymous=True)
    rate = rospy.Rate(10)

    ################# WHILE KEEP GETTING POSITION OF OBJECT W.R.T. ROBOT ##############
    distance = 0.5
    angle = 5

    commander(pub, distance, angle, rate)


