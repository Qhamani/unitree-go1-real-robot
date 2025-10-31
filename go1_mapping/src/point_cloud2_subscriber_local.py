#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

# Global publisher variable
pub = None 

def scan_callback(msg):
    # This is the core logic: update the timestamp
    msg.header.stamp = rospy.Time.now()
    
    # Publish the modified message
    if pub:
        pub.publish(msg)

if __name__ == '__main__':
    # 1. Initialize the node
    rospy.init_node("scan_republisher")
    
    # 2. Create the publisher
    pub = rospy.Publisher("/lidar_pc2", PointCloud2, queue_size=10)
    
    # 3. Create the subscriber
    rospy.Subscriber("/rslidar_points", PointCloud2, scan_callback)
    
    # 4. Remove the rospy.sleep(0.5) line. 
    # It is unnecessary and can sometimes cause problems.
    
    # 5. Enter the main loop to process messages (the callback function)
    rospy.spin()