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
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <math.h>



using namespace UNITREE_LEGGED_SDK;

ros::Publisher odom_pub;    // publisher for odometry

void baseOdomTF(nav_msgs::Odometry &odom);

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{

    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "go1_odom";  
    odom.child_frame_id = "base_link";
 

    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];

    odom.twist.twist.linear.x  = msg->velocity[0];
    odom.twist.twist.linear.y  = msg->velocity[1];
    odom.twist.twist.angular.z = msg->yawSpeed;
    
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(
                                                msg->imu.rpy[0],
                                                msg->imu.rpy[1],
                                                msg->imu.rpy[2]);
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);

    baseOdomTF(odom);
   
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

//---------------------------------------------------------
//              Base -> Odometry TF
//---------------------------------------------------------

void baseOdomTF(nav_msgs::Odometry &odom){

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom.header.stamp;
    odom_trans.header.frame_id = odom.header.frame_id;
    odom_trans.child_frame_id = odom.child_frame_id;

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    
    odom_trans.transform.rotation.x = odom.pose.pose.orientation.x;
    odom_trans.transform.rotation.y = odom.pose.pose.orientation.y;
    odom_trans.transform.rotation.z = odom.pose.pose.orientation.z;
    odom_trans.transform.rotation.w = odom.pose.pose.orientation.w;

    odom_broadcaster->sendTransform(odom_trans);
}

