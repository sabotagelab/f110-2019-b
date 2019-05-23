#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
import tf.transformations as transform
import time
import numpy as np
from race.msg import drive_param


class OdomTF(object):
    def __init__(self):
        rospy.init_node('odom_tf')
        rospy.Subscriber('drive_parameters', drive_param, self.drive_command_callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = tf.TransformBroadcaster()
        self.speed = 0
        self.angle =0
        self.x=0.0
        self.y=0.0
        self.yaw=0.0
        self.then = rospy.Time.now()

    def drive_command_callback(self, command):
        self.transform_position[0] = odom_pose.pose.position.x
        self.transform_position[1] = odom_pose.pose.position.y
        self.transform_quaternion[0] = odom_pose.pose.orientation.x
        self.transform_quaternion[1] = odom_pose.pose.orientation.y
        self.transform_quaternion[2] = odom_pose.pose.orientation.z
        self.transform_quaternion[3] = odom_pose.pose.orientation.w



    def update(self):
        now = rospy.Time.now()
        elapsed = now-self.then
        elapsed = elapsed.to_sec()
        self.then=now

        dx = self.speed * elapsed
        # dr = self.delta_th / elapsed

        # calculate distance traveled in x and y
        x =
        y = sin(self.yaw) * d
        # calculate the final position of the robot
        self.x = self.x + cos(self.th) * d
        self.y = self.y + y

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)


if __name__ == '__main__':
    odomtf = OdomTF()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        odomtf.update()
        r.sleep()