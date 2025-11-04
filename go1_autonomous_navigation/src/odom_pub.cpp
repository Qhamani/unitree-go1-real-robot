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

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


using namespace UNITREE_LEGGED_SDK;

ros::Publisher odom_pub;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{

    nav_msgs::Odometry odom;

    // object odom is structured :

    //-------------------------------------------------------
    //     std_msgs/Header header
    //   uint32 seq
    //   time stamp
    //   string frame_id
    // string child_frame_id
    // geometry_msgs/PoseWithCovariance pose
    //   geometry_msgs/Pose pose
    //     geometry_msgs/Point position
    //       float64 x
    //       float64 y
    //       float64 z
    //     geometry_msgs/Quaternion orientation
    //       float64 x
    //       float64 y
    //       float64 z
    //       float64 w
    //   float64[36] covariance
    // geometry_msgs/TwistWithCovariance twist
    //   geometry_msgs/Twist twist
    //     geometry_msgs/Vector3 linear
    //       float64 x
    //       float64 y
    //       float64 z
    //     geometry_msgs/Vector3 angular
    //       float64 x
    //       float64 y
    //       float64 z
    //   float64[36] covariance
    //---------------------------------------------------------


    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "go1_odom";  
    odom.child_frame_id = "base_link";
 

    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];
    

    odom.pose.pose.orientation.x = msg->imu.quaternion[1];
    odom.pose.pose.orientation.y = msg->imu.quaternion[2];
    odom.pose.pose.orientation.z = msg->imu.quaternion[3];
    odom.pose.pose.orientation.w = msg->imu.quaternion[0];


    odom.twist.twist.linear.x  = msg->velocity[0];
    odom.twist.twist.linear.y  = msg->velocity[1];
    odom.twist.twist.angular.z = msg->yawSpeed;

    odom_pub.publish(odom);
   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle nh;

    ros::Subscriber high_sub = nh.subscribe("high_state", 500, highStateCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/go1/odom", 500);

    ros::spin();

    return 0;
}


