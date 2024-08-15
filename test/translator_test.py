#!/usr/bin/env python3
import numpy as np

import rospy
import tf2_ros
from nav_msgs.msg import Odometry

class Translator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_update_talker', anonymous=True)
        
        self.pub = rospy.Publisher('/racetrack/odom_passthrough', Odometry, queue_size=20)
        self.sub = rospy.Subscriber('/compslam_lio/odometry', Odometry, self.compslam_callback)

        # TF listeners to run in the background
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Fixed republishing covariance
        self.covariance = np.eye(6).flatten() * 0.01

    def compslam_callback(self, odometry_msg):
        republish_msg = Odometry()
        republish_msg.pose.covariance = self.covariance

        if odometry_msg.child_frame_id != 'base':
            try:
                tf2_B_F = self.tf_buffer.lookup_transform(
                    odometry_msg.header.frame_id, "base", republish_msg.header.stamp, rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as error:
                print("[CybathlonPoseTranslator]: TF lookup failed.", error)
                return False

            republish_msg.header = odometry_msg.header
            republish_msg.child_frame_id = 'base'
            republish_msg.pose.pose.position = tf2_B_F.transform.translation
            republish_msg.pose.pose.orientation = tf2_B_F.transform.rotation
        else:
            republish_msg.header = odometry_msg.header
            republish_msg.pose.pose = odometry_msg.pose.pose

        self.pub.publish(republish_msg)

if __name__ == '__main__':
    try:
        translator = Translator()
        # translator.publish_pose_update()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
