#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def talker():

    publisher = rospy.Publisher("/base_controller/command", Twist, queue_size=10) # publishes message to the /base_controller/command
    rospy.init_node('fetch_driver', anonymous=True) # initializes the node called fetch_driver, makes sure its unique name
    rate = rospy.Rate(10) # 10 hz publishing rate

    while not rospy.is_shutdown():
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0

        rospy.loginfo('fetch_driver is moving the fetch robot ...')
        publisher.publish(msg)
        rate.sleep()


if __name__== "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
