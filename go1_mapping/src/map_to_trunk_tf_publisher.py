#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomToTFBroadcaster(object):
    def __init__(self):
        rospy.init_node('map_to_trunk_tf_publisher', anonymous=True)
        rospy.loginfo("Starting Map to Trunk TF Broadcaster...")

        self.br = tf2_ros.TransformBroadcaster()
        rospy.sleep(1.0)
        rospy.Subscriber('/ros2udp/odom', Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
        try:
            parent_frame = "map"
            child_frame = "trunk"

            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

            self.br.sendTransform(t)

        except Exception as e:
            rospy.logerr("Error processing Odometry message:")

if __name__ == '__main__':
    try:
        OdomToTFBroadcaster()
    except rospy.ROSInterruptException:
        pass



