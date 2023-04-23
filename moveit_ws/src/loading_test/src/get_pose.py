#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates


def callback(data):

    list_len = len(data.name) # including "world_plane"
    index = 4

    rospy.loginfo(rospy.get_caller_id() + " -> The %s has position (%.3f, %.3f, %.3f)", 
                (data.name)[index], (data.pose)[index].position.x, (data.pose)[index].position.y, (data.pose)[index].position.z)


def listener():

    rospy.init_node('pose_listener', anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size=1, buff_size=2**24)

    rospy.spin()


if __name__ == '__main__':
    listener()
