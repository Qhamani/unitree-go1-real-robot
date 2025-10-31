#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped , PoseStamped
import tf_conversions

def pose_callback(msg):
    # 1. Create a TransformStamped message
    t = TransformStamped()
    msg2 = PoseStamped()

    # 2. Set the metadata
    #t.header.stamp = rospy.Time.now()
    # t.header.stamp = msg.header.stamp
    t.header.frame_id = "map"          # This is the parent frame
    t.child_frame_id = "trunk"     # This is the child (robot) frame

    # 3. Set the translation (x and y)
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    # 4. Convert yaw (theta) to a quaternion
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # 5. Broadcast the transform
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('pose2d_to_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/pose2D", Pose2D, pose_callback)
    rospy.spin()