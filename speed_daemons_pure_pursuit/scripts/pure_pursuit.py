#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import tf.transformations as transform
from pure_pursuit_utils import *
import copy


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pf_pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.LOOKAHEAD_DISTANCE = 2.0  # meters
        self.VELOCITY = 1.0  # m/s
        filename = rospy.get_param('~waypoints_filepath', '')
        self.waypoints = read_waypoints_from_csv(filename)
        self.current_pose = [0,0,0]
        self.last_search_index = 0
        print(len(self.waypoints))
        print(self.waypoints[0])


    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Runs pure pursuit and publishes velocity and steering angle.
    def pf_pose_callback(self,msg):
        # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.
        # 1. Determine the current location of the vehicle

        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        pose_yaw = quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose = [pose_x,pose_y,pose_yaw]
        print "pose:=", pose_x, pose_y ,pose_yaw

        # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.

        # goal_point = None
        # for point in reversed(self.waypoints):
        #     goal_point = point
        #     distance = dist(point,self.current_pose)
        #     if distance <= self.LOOKAHEAD_DISTANCE:
        #         break

        distance = 0.0
        last_distance = 0
        second_last_index = len(self.waypoints)-2
        if self.last_search_index != (second_last_index+1):
            for index, point in enumerate(self.waypoints):
                if index < self.last_search_index:
                    continue
                if self.last_search_index == second_last_index:
                    self.last_search_index = second_last_index + 1
                    break
                distance = dist(point,self.current_pose)
                if last_distance <= self.LOOKAHEAD_DISTANCE < distance:
                    self.last_search_index = index-1
                    break
                last_distance = copy.deepcopy(distance)

        print last_distance,distance, self.waypoints[self.last_search_index],self.waypoints[self.last_search_index+1]
        goal_point = self.waypoints[self.last_search_index]
        print "goal-point:=", goal_point

        # 3. Transform the goal point to vehicle coordinates.
        beta = math.atan2((goal_point[0] - self.current_pose[0]), (goal_point[1] - self.current_pose[1]))
        gamma = math.pi / 2 - pose_yaw - beta
        point_x_wrt_car = -1.0 * distance * math.sin(gamma)
        point_y_wrt_car = distance * math.cos(gamma)
        local_pose = (point_x_wrt_car,point_y_wrt_car,0)
        print "local pose:=",local_pose

        # 4. Calculate the curvature = 1/r = 2x/l^2
        # The curvature is transformed into steering wheel angle by the vehicle on board controller.
        # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
        angle = -2.0 * point_x_wrt_car / distance ** 2
        angle = constrain(angle,math.radians(-30),math.radians(30))

        msg = drive_param()
        msg.velocity = self.VELOCITY
        msg.angle = angle
        self.drive_pub.publish(msg)
    
if __name__ == '__main__':
    pure_pursuit = PurePursuit()
    rospy.spin()

