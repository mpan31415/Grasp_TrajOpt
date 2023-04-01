#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>

void quaternion_to_euler(double qw, double qx, double qy, double qz, double &roll, double &pitch, double &yaw) {
    // roll (x-axis rotation)
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny, cosy);
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int robot_pos[3];
    int robot_quart[4];
    int robot_euler[3];
    for (int i = 0; i < msg->name.size(); i++)
    {
        double qx = msg->pose[i].orientation.x;
        double qy = msg->pose[i].orientation.y;
        double qz = msg->pose[i].orientation.z;
        double qw = msg->pose[i].orientation.w;
        double roll, pitch, yaw;
        quaternion_to_euler(qw, qx, qy, qz, roll, pitch, yaw);
        ROS_INFO("Model name: %s", msg->name[i].c_str());
        ROS_INFO("Model pose: %f %f %f", msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
        ROS_INFO("Model orientation: %f %f %f %f", msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);
        ROS_INFO("Model roll pitch yaw: %f %f %f", roll, pitch, yaw);
    }

    ROS_INFO("\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_states_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
    ros::spin();
    return 0;
}