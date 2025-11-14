#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher cloud_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud = *msg;      // copy incoming message
    cloud.header.stamp = ros::Time::now();      // fix timestamp
    cloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_timestamp_fixer");
    ros::NodeHandle nh;

    // Subscribe to raw LIDAR points
    ros::Subscriber cloud_sub = nh.subscribe("/rslidar_points", 10, cloudCallback);

    // Republish with fixed timestamp
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_stamped", 10);

    ros::spin();
    return 0;
}
