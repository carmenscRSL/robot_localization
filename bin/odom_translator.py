#!/usr/bin/env python3
import numpy as np
import quaternion

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class Translator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_update_talker', anonymous=True)
        
        self.pub = rospy.Publisher('/racetrack/odom_passthrough', Odometry, queue_size=20)
        self.sub_odometry = rospy.Subscriber('/compslam_lio/odometry', Odometry, self.compslam_callback)
        self.sub_filtered = rospy.Subscriber('/odometry/filtered', Odometry, self.filtered_callback)

        # TF listeners to run in the background
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Fixed republishing covariance
        self.covariance = np.eye(6).flatten() * 0.01

    def compslam_callback(self, odometry_msg):
        republish_msg = Odometry()
        republish_msg.pose.covariance = self.covariance

        if odometry_msg.child_frame_id != 'base':
            try:
                tf2_O_B = self.tf_buffer.lookup_transform(
                    odometry_msg.header.frame_id, "base", odometry_msg.header.stamp, rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                print("[CybathlonPoseTranslator]: TF lookup failed.", error)
                return False

            republish_msg.header = odometry_msg.header
            republish_msg.child_frame_id = 'base'
            republish_msg.pose.pose.position = tf2_O_B.transform.translation
            republish_msg.pose.pose.orientation = tf2_O_B.transform.rotation
        else:
            republish_msg.header = odometry_msg.header
            republish_msg.pose.pose = odometry_msg.pose.pose

        self.pub.publish(republish_msg)

    def filtered_callback(self, odometry_msg):
        q_W_B = np.quaternion(
            odometry_msg.pose.pose.orientation.w,
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z
        )

        T_W_B = np.eye(4)
        T_W_B[0:3, 0:3] = quaternion.as_rotation_matrix(q_W_B)
        T_W_B[0, 3] = odometry_msg.pose.pose.position.x
        T_W_B[1, 3] = odometry_msg.pose.pose.position.y
        T_W_B[2, 3] = odometry_msg.pose.pose.position.z

        T_B_W = np.linalg.inv(T_W_B)

        t = TransformStamped()
        t.header.stamp = odometry_msg.header.stamp
        t.header.frame_id = "base"
        t.child_frame_id = "racetrack"
        t.transform.translation.x = T_B_W[0, 3]
        t.transform.translation.y = T_B_W[1, 3]
        t.transform.translation.z = T_B_W[2, 3]
        q_B_W = quaternion.from_rotation_matrix(T_B_W[:3, :3])
        t.transform.rotation.x = q_B_W.x
        t.transform.rotation.y = q_B_W.y
        t.transform.rotation.z = q_B_W.z
        t.transform.rotation.w = q_B_W.w
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        translator = Translator()
        # translator.publish_pose_update()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
