#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include "convert.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

// Global variable to store latest cmd_vel message
geometry_msgs::Twist latest_cmd_vel;
bool isInitPose = false;

// Callback: updates velocity command
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    latest_cmd_vel = *msg;
    ROS_INFO("Received cmd_vel -> Linear[x: %.2f, y: %.2f] | Angular[z: %.2f]",
             msg->linear.x, msg->linear.y, msg->angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_cmdvel_bridge");
    ros::NodeHandle nh;

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::Rate loop_rate(500);


    unitree_legged_msgs::HighCmd high_cmd_ros;
    long motiontime = 0;

    // Publisher to Unitree command topic
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    // Subscriber to cmd_vel topic
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);

    ROS_INFO("Node go1_cmdvel_bridge started. Listening to /cmd_vel...");

    while (ros::ok())
    {
        if (!isInitPose){
            high_cmd_ros.head[0] = 0xFE;
            high_cmd_ros.head[1] = 0xEF;
            high_cmd_ros.levelFlag = HIGHLEVEL;
            high_cmd_ros.mode = 0;
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.speedLevel = 0;
            high_cmd_ros.footRaiseHeight = 0;
            high_cmd_ros.bodyHeight = 0;
            high_cmd_ros.euler[0] = 0;
            high_cmd_ros.euler[1] = 0;
            high_cmd_ros.euler[2] = 0;
            high_cmd_ros.velocity[0] = 0.0f;
            high_cmd_ros.velocity[1] = 0.0f;
            high_cmd_ros.yawSpeed = 0.0f;
            high_cmd_ros.reserve = 0;

            isInitPose = true;
            ROS_INFO("Initialised Pose");
        }

        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.velocity[0] = static_cast<float>(latest_cmd_vel.linear.x);   // forward/backward
        high_cmd_ros.velocity[1] = static_cast<float>(latest_cmd_vel.linear.y);   // lateral
        high_cmd_ros.yawSpeed   = static_cast<float>(latest_cmd_vel.angular.z);   // rotation
        high_cmd_ros.bodyHeight = 0.1;

        // Safety stop: if cmd_vel not updated for some time, zero velocities
        // (optional, could add a timestamp check here later)

        pub.publish(high_cmd_ros);

        ROS_INFO("Published");

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
