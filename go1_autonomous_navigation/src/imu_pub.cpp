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
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


using namespace UNITREE_LEGGED_SDK;

ros::Publisher imu_pub;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{

    sensor_msgs::Imu imu;

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link";  
 
    imu.orientation.x = msg->imu.quaternion[1];
    imu.orientation.x = msg->imu.quaternion[2];
    imu.orientation.x = msg->imu.quaternion[3];
    imu.orientation.x = msg->imu.quaternion[0];

    imu.angular_velocity.x = msg->imu.gyroscope[0];
    imu.angular_velocity.y = msg->imu.gyroscope[1];
    imu.angular_velocity.z = msg->imu.gyroscope[2];


    imu.linear_acceleration.x = msg->imu.accelerometer[0];
    imu.linear_acceleration.y = msg->imu.accelerometer[1];
    imu.linear_acceleration.z = msg->imu.accelerometer[2];

    imu_pub.publish(imu);
   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle nh;

    ros::Subscriber high_sub = nh.subscribe("high_state", 500, highStateCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/go1/imu", 500);

    ros::spin();

    return 0;
}


