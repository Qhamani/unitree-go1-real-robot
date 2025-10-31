#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace UNITREE_LEGGED_SDK;

ros::Publisher odom_pub;


//----------------------------------------------------------
//          helper functions for odometry
//--------------------------------------------------------

// nav_msgs get_position(const unitree_legged_msgs::HighState::ConstPtr &high_state) {
//     return {high_state.position[0],
//             high_state.position[1], 
//             high_state.position[2]};
// }

// nav_msgs get_quaternion(const unitree_legged_msgs::HighState::ConstPtr &high_state) {
//     return {high_state.imu.quaternion[1],
//             high_state.imu.quaternion[2],
//             high_state.imu.quaternion[3],
//             high_state.imu.quaternion[0]};
// }

// nav_msgs get_velocity(const unitree_legged_msgs::HighState::ConstPtr &high_state) {
//     return {high_state.velocity[0], 
//             high_state.velocity[1], 
//             high_state.yawSpeed};
// }



void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{

    nav_msgs::Odometry odom;

    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];

    odom.pose.pose.orientation.x = msg->imu.quaternion[1];
    odom.pose.pose.orientation.y = msg->imu.quaternion[2];
    odom.pose.pose.orientation.z = msg->imu.quaternion[3];
    odom.pose.pose.orientation.w = msg->imu.quaternion[0]; 

    odom.twist.twist.linear.x  = msg->velocity[0];
    odom.twist.twist.linear.y  = msg->velocity[1];
    odom.twist.twist.linear.z  = msg->velocity[2];
    odom.twist.twist.angular.z = msg->yawSpeed;

    odom_pub.publish(odom);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle nh;

    unitree_legged_msgs::HighState high_state_ros;

    ros::Subscriber high_sub = nh.subscribe("high_state", 10, highStateCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::spin();

    return 0;
}


