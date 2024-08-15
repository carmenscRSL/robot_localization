#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry

class Translator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_update_talker', anonymous=True)
        
        # Publisher
        self.pub = rospy.Publisher('/racetrack/odom_passthrough', Odometry, queue_size=20)
        
        # Subscriber
        self.sub = rospy.Subscriber('/compslam_lio/odometry', Odometry, self.compslam_callback)

        self.covariance = np.eye(6).flatten() * 0.01

    def compslam_callback(self, alma_msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = alma_msg.header.stamp
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base"
        odom_msg.pose.pose = alma_msg.pose.pose
        odom_msg.pose.covariance = self.covariance
        # odom_msg.twist.twist = alma_msg.twist.twist
        # odom_msg.twist.covariance = self.covariance

        self.pub.publish(odom_msg)


if __name__ == '__main__':
    try:
        translator = Translator()
        # translator.publish_pose_update()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
