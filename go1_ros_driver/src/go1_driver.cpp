#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "convert.h"
#include <pthread.h>
#include <chrono>

#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "common.h"

using namespace UNITREE_LEGGED_SDK;

class Go1Driver {
public:

    bool got_state = false;
    bool running = false;

    Go1Driver()
        : high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)),
        loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Go1Driver::highUdpSend, this)),
        loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Go1Driver::highUdpRecv, this))
    {
        high_udp.InitCmdData(high_cmd);

        odom_pub = nh.advertise<nav_msgs::Odometry>("/go1/odom", 10);
        imu_pub = nh.advertise<sensor_msgs::Imu>("/go1/imu", 10);
        high_state_pub = nh.advertise<unitree_legged_msgs::HighState>("/high_state", 10);
        high_cmd_sub = nh.subscribe("/high_cmd", 10, &Go1Driver::highCmdCallback, this);

        loop_udpSend.start();
        loop_udpRecv.start();
        running = true;
    }



    ~Go1Driver() {
        running = false; // will use it later to clean up memory ( but does nothing for now)
    }

    
    // getter for latest high_state mesage
    unitree_legged_msgs::HighState getHighState()
    {
        return high_state_msg;
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

        baseOdomTF(odom);
    }

    //---------------------------------------------------------
    //              IMU Pub
    //---------------------------------------------------------

    void publishIMU(const unitree_legged_msgs::HighState &msg)
    
    {

        sensor_msgs::Imu imu;

        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";  
    
        imu.orientation.x = msg.imu.quaternion[1];
        imu.orientation.y = msg.imu.quaternion[2];
        imu.orientation.z = msg.imu.quaternion[3];
        imu.orientation.w = msg.imu.quaternion[0];

        imu.angular_velocity.x = msg.imu.gyroscope[0];
        imu.angular_velocity.y = msg.imu.gyroscope[1];
        imu.angular_velocity.z = msg.imu.gyroscope[2];


        imu.linear_acceleration.x = msg.imu.accelerometer[0];
        imu.linear_acceleration.y = msg.imu.accelerometer[1];
        imu.linear_acceleration.z = msg.imu.accelerometer[2];

        imu_pub.publish(imu);
   
    }


private:
    ros::NodeHandle nh;
    ros::Publisher odom_pub, high_state_pub, imu_pub;
    ros::Subscriber high_cmd_sub, cmd_vel;
    tf::TransformBroadcaster odom_broadcaster;
   
    LoopFunc loop_udpSend ,loop_udpRecv;

    UDP high_udp;
    HighCmd high_cmd = {0};
    HighState high_state = {0};
    unitree_legged_msgs::HighState high_state_msg;

    void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg) {
        
        this->high_cmd = rosMsg2Cmd(msg);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void highUdpRecv()
    {
        // printf("high udp reccv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);

        high_state_msg = state2rosMsg(high_state);  // storing the latest highstate (to consume at lower rate 50 Hz << 500 Hz)
        got_state = true;  
        high_state_pub.publish(high_state_msg);    // publish ros highstate msg ( 500Hz - from thread)
    }


    //---------------------------------------------------------
    //              Base -> Odometry TF
    //---------------------------------------------------------

    void baseOdomTF(nav_msgs::Odometry &odom)
    
    {

        geometry_msgs::TransformStamped odom_trans;
        
        odom_trans.header.stamp = odom.header.stamp;
        odom_trans.header.frame_id = odom.header.frame_id;
        odom_trans.child_frame_id = odom.child_frame_id;

        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.translation.z = odom.pose.pose.position.z;
        
        odom_trans.transform.rotation = odom.pose.pose.orientation;

        odom_broadcaster.sendTransform(odom_trans);
    }


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_driver");
    Go1Driver driver;

    ros::Rate rate(50.0); 
    while (ros::ok())
    {
        ros::spinOnce();
        if (driver.got_state){
            driver.publishOdometry(driver.getHighState());
            driver.publishIMU(driver.getHighState());
        }
        rate.sleep();
    }
    return 0;
}
