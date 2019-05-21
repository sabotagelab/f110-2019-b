#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Float32, String, ColorRGBA, Header
import tf.transformations as transform
from race.msg import drive_param
from speed_daemons_wall_following.msg import turn_instruction
import math


class Visualizer:
    def __init__(self):
        rospy.init_node('wall_following_visualizer')
        # We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
        self.publisher_wall_alignment = rospy.Publisher('/visualization_wall_alignment', Marker, queue_size=1)
        self.publisher_turns = rospy.Publisher('/visualization_turns', Marker, queue_size=1)
        self.publisher_instruction = rospy.Publisher('/visualization_instruction', Marker, queue_size=1)
        self.publisher_velocity = rospy.Publisher('/visualization_velocity', Marker, queue_size=1)
        self.publisher_actual_velocity = rospy.Publisher('/visualization_actual_velocity', Marker, queue_size=1)
        rospy.Subscriber('turning_mode', String, self.turning_callback)
        rospy.Subscriber("instruction_feedback", String, self.turn_status_callback)
        rospy.Subscriber("drive_vel", Float32, self.velocity_callback)
        rospy.Subscriber('drive_parameters', drive_param, self.actual_speed_callback)
        rospy.Subscriber('drive_instruction', turn_instruction, self.instruction_callback)
        # default wall following operation mode is right
        self.wallAlignment = rospy.get_param('wall_follow_side', 'right')
        self.turnStatus = "turnCompleted"
        vel = rospy.get_param('default_velocity', 2.0)
        self.velocity = "Speed: " + str(vel) + " m/s"
        self.actual_velocity = str(0.0)
        self.current_instruction = "Instruction: " + rospy.get_param('wall_follow_side', ' ')

    def instruction_callback(self, msg):
        self.current_instruction = "Instruction: " + msg.command

    def turn_status_callback(self, data):
        if data.data == "turnCompleted":
            self.turnStatus = data.data

    def turning_callback(self, data):
        self.wallAlignment = data.data
        self.turnStatus = "Turning Now"

    def velocity_callback(self, data):
        self.velocity = "Speed: " + str(data.data) + " m/s"

    def actual_speed_callback(self, data):
        self.actual_velocity = "True speed:" + str(data.velocity) + " m/s"

    def publish_text(self, text, x, y, publisher):
        marker = Marker()
        # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
        marker.header.frame_id = "/base_link"
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0  # or set this to 0
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.lifetime = rospy.Duration(0.1)
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        publisher.publish(marker)  # Publish the MarkerArray

    def publish_arrow(self, direction, publisher):
        dir_marker = Marker()
        dir_marker.type = Marker.ARROW
        dir_marker.id = 0
        dir_marker.lifetime = rospy.Duration(0.2)

        if direction == 'left':
            dir_marker.pose.position = Point(1, 0, 0)
            quaternion = transform.quaternion_from_euler(0, 0, math.radians(90))
            dir_marker.pose.orientation.z = quaternion[2]
            dir_marker.pose.orientation.w = quaternion[3]
        elif direction == 'right':
            dir_marker.pose.position = Point(1, 0, 0)
            quaternion = transform.quaternion_from_euler(0, 0, math.radians(-90))
            dir_marker.pose.orientation.z = quaternion[2]
            dir_marker.pose.orientation.w = quaternion[3]

        dir_marker.scale = Vector3(0.56, 0.06, 0.06)
        dir_marker.header = Header(frame_id='base_link')
        dir_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
        publisher.publish(dir_marker)


if __name__ == '__main__':
    visualizer = Visualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer.publish_text(visualizer.turnStatus, -1, 1, visualizer.publisher_wall_alignment)
        visualizer.publish_text(visualizer.current_instruction, 0, 1, visualizer.publisher_instruction)
        visualizer.publish_arrow(visualizer.wallAlignment, visualizer.publisher_turns)
        visualizer.publish_text(visualizer.velocity, 0, -2, visualizer.publisher_velocity)
        visualizer.publish_text(visualizer.actual_velocity, -1, -2, visualizer.publisher_actual_velocity)
        rate.sleep()
