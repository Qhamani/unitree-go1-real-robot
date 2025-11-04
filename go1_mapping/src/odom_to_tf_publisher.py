#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class OdomToTFBroadcaster:
    def __init__(self):
        rospy.init_node('odom_to_tf_publisher', anonymous=True)
        rospy.loginfo("Starting Odometry-to-TF broadcaster (using tf)...")

        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("/go1/odom", Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
        try:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            t = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

            # Broadcast transform: go1odom -> mybase
            self.br.sendTransform(
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                t,
                "base_link",     # child frame
                "go1_odom"     # parent frame
            )

        except Exception as e:
            rospy.logwarn("Error broadcasting TF: %s" % str(e))



if __name__ == '__main__':
    try:
        OdomToTFBroadcaster()
    except rospy.ROSInterruptException:
        pass
