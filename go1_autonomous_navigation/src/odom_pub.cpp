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
tf::TransformBroadcaster* odom_broadcaster;

void baseOdomTF(nav_msgs::Odometry &odom);
void publishOdometry(const unitree_legged_msgs::HighState &msg);


bool got_state = false;
unitree_legged_msgs::HighState latest_state;

// Callback just stores latest state
void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{
    latest_state = *msg;
    got_state = true;
    printf("Highstate callback");
}

void publishOdometry(const unitree_legged_msgs::HighState &msg)
{

    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "go1_odom";  
    odom.child_frame_id = "base_link";
 

    odom.pose.pose.position.x = msg.position[0];
    odom.pose.pose.position.y = msg.position[1];
    odom.pose.pose.position.z = msg.position[2];

    odom.twist.twist.linear.x  = msg.velocity[0];
    odom.twist.twist.linear.y  = msg.velocity[1];
    odom.twist.twist.angular.z = msg.yawSpeed;
    
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(
                                                msg.imu.rpy[0],
                                                msg.imu.rpy[1],
                                                msg.imu.rpy[2]);
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
    printf("odometry");

    baseOdomTF(odom);
   
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
    
    odom_trans.transform.rotation = odom.pose.pose.orientation;

    odom_broadcaster->sendTransform(odom_trans);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle nh;

    ros::Subscriber high_sub = nh.subscribe("high_state", 10, highStateCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/go1/odom", 10);
    odom_broadcaster = new tf::TransformBroadcaster();

    ros::Rate rate(50.0); 
    while (ros::ok())
    {
        ros::spinOnce();
        if (got_state)
            publishOdometry(latest_state);
        rate.sleep();
    }

    return 0;
}


