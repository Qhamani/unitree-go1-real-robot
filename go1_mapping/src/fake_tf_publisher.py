#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

# Define the parent and child frame IDs
ODOM_FRAME_ID = "odom"
BASE_FRAME_ID = "trunk" # Use 'trunk' instead of 'base_link' as per your robot's URDF

# Global broadcaster object
br = None

def odom_callback(odom_msg):
    """
    Callback function to receive Odometry messages and publish the TF transform.
    """
    if br is None:
        rospy.logerr("TF Broadcaster not initialized.")
        return

    # Extract the position and orientation data from the Odometry message
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    
    # Create the transform (translation and rotation)
    translation = (p.x, p.y, p.z)
    rotation = (q.x, q.y, q.z, q.w)

    # Publish the transform
    # The transform is from the parent frame (odom) to the child frame (trunk)
    br.sendTransform(
        translation,
        rotation,
        odom_msg.header.stamp,  # Use the timestamp from the Odometry message
        BASE_FRAME_ID,
        ODOM_FRAME_ID
    )

def odom_tf_broadcaster():
    global br
    # Initialize the ROS node
    rospy.init_node('odom_tf_broadcaster', anonymous=True)

    # Create the Transform Broadcaster
    br = tf.TransformBroadcaster()

    # Subscribe to the /odom topic
    rospy.Subscriber('odom', Odometry, odom_callback)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass