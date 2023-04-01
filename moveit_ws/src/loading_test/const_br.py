#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':

    rospy.init_node("fixed_tf_broadcaster")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    # Rubix's cube
    x_1 = 0
    # y_1 = 0.025
    y_1 = 0.012
    z_1 = -0.02

    # Shelf
    x_4 = -(0.595 + 0.55)
    y_4 = 0.275 + 0.10
    z_4 = -1.094 + 0.027 - 0.01

    # table
    # x_0 = 0
    # y_0 = 0    
    # z_0 = 0

    # Honey
    x_7 = 0
    # y_7 = 0.04
    y_7 = 0.025
    z_7 = -0.01

    while not rospy.is_shutdown():

        br.sendTransform([x_4, y_4, z_4], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), "shelf", "ar_marker_8")
        br.sendTransform([x_1, y_1, z_1], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), "cube_2", "ar_marker_1")
        br.sendTransform([x_7, y_7, z_7], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), "cube_1", "ar_marker_7")

        # br.sendTransform([x_0, y_0, z_0], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), "table", "ar_marker_0")

        rate.sleep()
