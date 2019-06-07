#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import numpy as np
import tf.transformations as transform
from pure_pursuit_utils import *
from speed_daemons_pure_pursuit.msg import pure_pursuit_param
import copy


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        self.current_pose = [0, 0, 0]
        self.last_search_index = 0
        self.FIND_NEAREST_POINT = True

        filename = rospy.get_param('~waypoints_filepath', '')
        self.waypoints = read_waypoints_from_csv(filename)

        self.LOOKAHEAD_DISTANCE = rospy.get_param('initial_lookahead_distance', 1.5)
        self.VELOCITY = rospy.get_param('initial_velocity', 2.0)
        self.LOOP_ENABLE = rospy.get_param('loop_enable', False)
        self.SPEED_CONTROL = rospy.get_param('speed_control', False)
        self.STRAIGHT_LOOKAHEAD_DIST = rospy.get_param('straight_lookahead_dist', 3.0)
        self.TURNING_LOOKAHEAD_DIST = rospy.get_param('turning_lookahead_dist', 0.75)
        self.STRAIGHT_VEL = rospy.get_param('straight_vel', 2.0)
        self.TURNING_VEL = rospy.get_param('turning_vel', 0.75)
        self.TURN_WINDOW_MIN = rospy.get_param('turn_window_min', 3.75)
        self.TURN_WINDOW_MAX = rospy.get_param('turn_window_max', 4.5)
        self.MAX_X_DEVIATION = rospy.get_param('max_x_deviation', 4.75)
        self.RATE = rospy.get_param('pure_pursuit_rate', 20)
        self.INTERPOLATION_MODE = rospy.get_param('interpolation_mode', 'linear')
        print(self.INTERPOLATION_MODE)
        self.kp_quad = rospy.get_param('kp_quad', 0.75)
        self.quad_factor = rospy.get_param('quad_factor', 2.5)
        self.NEAR_END_OF_WAY_POINTS = False
        self.pose_read_flag = False
        rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pf_pose_callback, queue_size=1)
        rospy.Subscriber('pure_pursuit_reset', Bool, self.joy_reset_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.parameters_pub = rospy.Publisher('pure_pursuit_parameters', pure_pursuit_param, queue_size=1)


    def find_closed_point_index(self, current_pos):
        nearest_index = 0
        nearest_dist = float('inf')
        for index, point in enumerate(self.waypoints):
            distance = dist(current_pos, point)
            if distance <= nearest_dist:
                nearest_dist = distance
                nearest_index = index
        return nearest_index

    def joy_reset_callback(self, msg):
        if msg.data:
            self.FIND_NEAREST_POINT = True

    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Runs pure pursuit and publishes velocity and steering angle.
    def pf_pose_callback(self, msg):
        # Following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.
        # 1. Determine the current location of the vehicle
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        pose_yaw = quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose = [pose_x, pose_y, pose_yaw]
        if not self.pose_read_flag:
            self.pose_read_flag = True

    def do_pure_pursuit(self):
        # print "pose:=", pose_x, pose_y, pose_yaw
        if self.pose_read_flag:
            if self.FIND_NEAREST_POINT:
                self.last_search_index = self.find_closed_point_index(self.current_pose)
                print "Nearest index=", self.last_search_index
                self.FIND_NEAREST_POINT = False

            if self.LOOP_ENABLE and self.NEAR_END_OF_WAY_POINTS:
                self.last_search_index = 0
                self.NEAR_END_OF_WAY_POINTS = False

            if self.SPEED_CONTROL:
                average_x_from_car = self.get_x_deviation_in_path(self.current_pose[0], self.current_pose[1],
                                                                  self.current_pose[2])
                if self.INTERPOLATION_MODE == 'quadratic':
                    self.LOOKAHEAD_DISTANCE = self.do_quadratic_interpolation(average_x_from_car)
                else:
                    self.LOOKAHEAD_DISTANCE = np.interp(average_x_from_car, [0.0, self.MAX_X_DEVIATION],
                                                        [self.STRAIGHT_LOOKAHEAD_DIST, self.TURNING_LOOKAHEAD_DIST])

                self.VELOCITY = np.interp(average_x_from_car, [0.0, self.MAX_X_DEVIATION],
                                          [self.STRAIGHT_VEL, self.TURNING_VEL])


            pursuit_param = pure_pursuit_param()
            pursuit_param.lookahead_dist = self.LOOKAHEAD_DISTANCE
            pursuit_param.velocity = self.VELOCITY
            self.parameters_pub.publish(pursuit_param)

            # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.

            distance = 0.00
            last_index = len(self.waypoints) - 1

            for index, point in enumerate(self.waypoints):
                if index < self.last_search_index:
                    continue

                distance = dist(point, self.current_pose)
                if distance >= self.LOOKAHEAD_DISTANCE:
                    self.last_search_index = index
                    break

            if (last_index - 10) <= self.last_search_index <= last_index:
                self.NEAR_END_OF_WAY_POINTS = True

            goal_point = self.waypoints[self.last_search_index]
            # print  distance, self.waypoints[self.last_search_index]
            # print "goal-point:=", goal_point

            # 3. Transform the goal point to vehicle coordinates.
            point_x_wrt_car = self.compute_x_wrt_car(goal_point[0] - self.current_pose[0],
                                                     goal_point[1] - self.current_pose[1], self.current_pose[2],
                                                     distance)

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Flipping to negative because for the VESC a right steering angle has a negative value.

            # radius = (distance ** 2) / (-2.0 * point_x_wrt_car)
            # angular_velocity = self.VELOCITY / radius
            # theta = angular_velocity * self.RATE
            # angle = constrain(theta, math.radians(-30), math.radians(30))

            angle = -2.0 * point_x_wrt_car / distance ** 2
            angle = constrain(angle, math.radians(-30), math.radians(30))

            msg = drive_param()
            msg.velocity = self.VELOCITY
            msg.angle = angle
            self.drive_pub.publish(msg)

    def get_x_deviation_in_path(self, pose_x, pose_y, yaw):
        average_x_from_car = 0.0
        window_points_count = 0

        for index, point in enumerate(self.waypoints):
            if index < self.last_search_index:
                continue
            distance = dist(point, self.current_pose)
            if self.TURN_WINDOW_MIN <= distance <= self.TURN_WINDOW_MAX:
                average_x_from_car += self.compute_x_wrt_car(point[0] - pose_x, point[1] - pose_y, yaw, distance)
                window_points_count += 1

            if distance > self.TURN_WINDOW_MAX:
                break

        if window_points_count == 0:
            return 0.0
        average_x_from_car = average_x_from_car / window_points_count
        average_x_from_car = np.clip(abs(average_x_from_car), 0.0, self.MAX_X_DEVIATION)
        return average_x_from_car

    def compute_x_wrt_car(self, deltax, deltay, yaw, distance):
        beta = math.atan2(deltax, deltay)
        gamma = math.pi / 2 - yaw - beta
        x_wrt_car = -1.0 * distance * math.sin(gamma)
        return x_wrt_car

    def do_quadratic_interpolation(self,error):
        lookahead_response = self.STRAIGHT_LOOKAHEAD_DIST - self.kp_quad * pow(error,self.quad_factor)
        lookahead_response=np.clip(lookahead_response,self.TURNING_LOOKAHEAD_DIST,self.STRAIGHT_LOOKAHEAD_DIST)
        return lookahead_response

    def run_pure_pursuit(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            self.do_pure_pursuit()
            rate.sleep()


if __name__ == '__main__':
    pure_pursuit = PurePursuit()
    input('str')
    pure_pursuit.run_pure_pursuit()
    rospy.spin()
