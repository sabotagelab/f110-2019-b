#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from speed_daemons_wall_following.msg import wall_parameters
from race.msg import drive_param
import pdb


class PIDerror:
    def __init__(self):
        rospy.init_node('pid_error_node', anonymous=True)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("turning_mode", String, self.turning_callback)
        rospy.Subscriber('drive_parameters', drive_param, self.actual_speed_callback)
        self.pid_pub = rospy.Publisher('pid_error', Float64, queue_size=10)
        self.wall_data_pub = rospy.Publisher('wall_parameters', wall_parameters, queue_size=1)
        self.MIN_ANGLE = rospy.get_param('lidar_min_angle', 0.0)
        self.MAX_ANGLE = rospy.get_param('lidar_max_angle', 180.0)
        self.FOV = self.MAX_ANGLE - self.MIN_ANGLE
        self.WALL_SCAN_ANGLE = rospy.get_param('wall_scan_angle', 60.0)
        self.WALL_SCAN_ANGLE_RADIANS=math.radians(self.WALL_SCAN_ANGLE)
        self.alpha = 0                  #heading error with parallel wall
        self.TARGET_DISTANCE_RIGHT = rospy.get_param('target_distance_right', 1.0)
        self.TARGET_DISTANCE_LEFT = rospy.get_param('target_distance_left', 1.0)
        self.OPERATION_MODE = rospy.get_param('wall_follow_side', 'right')  # left, right and center
        self.DEBUG_MODE = rospy.get_param('debug_print_mode', False)
        self.current_dist = 0.0
        self.NORMAL_VEL = rospy.get_param('default_velocity', 2.0)
        self.max_lookahead_dist = rospy.get_param('max_lookahead_distance', 0.75)
        self.min_lookahead_dist = rospy.get_param('min_lookahead_distance', 0.35)
        self.adaptive_lookahead_flag = rospy.get_param('adaptive_lookahead', False)
        self.LOOKAHEAD_DIST = rospy.get_param('pid_lookahead_distance', 0.35)
        self.adapt_lookahead_distance()
        print "operation-mode=",self.OPERATION_MODE

    def adapt_lookahead_distance(self):
        if self.adaptive_lookahead_flag:
            self.LOOKAHEAD_DIST= self.NORMAL_VEL * 0.3
            if self.LOOKAHEAD_DIST > self.max_lookahead_dist:
                self.LOOKAHEAD_DIST = self.max_lookahead_dist
            elif self.LOOKAHEAD_DIST <= self.min_lookahead_dist:
                self.LOOKAHEAD_DIST = self.min_lookahead_dist
            # print(self.LOOKAHEAD_DIST)
        else:
            self.LOOKAHEAD_DIST = rospy.get_param('pid_lookahead_distance', 0.35)


    def actual_speed_callback(self,data):
        self.NORMAL_VEL =data.velocity
        self.adapt_lookahead_distance()


    def getRange(self,data, angle):
        range_idx = len(data.ranges) * (angle - self.MIN_ANGLE) / self.FOV

        range = data.ranges[int(range_idx)]
        if math.isinf(range):
            return data.range_max
        elif math.isnan(range):
            return self.getRange(data,angle-1)
        return range

    def followLeft(self,data, desired_distance):
        a = self.getRange(data, 180-self.WALL_SCAN_ANGLE)
        b = self.getRange(data, 179)
        self.alpha = -math.atan((a * math.cos(self.WALL_SCAN_ANGLE_RADIANS) - b) / (a * math.sin(self.WALL_SCAN_ANGLE_RADIANS)))
        self.current_dist = b * math.cos(self.alpha)
        future_dist = self.current_dist - self.LOOKAHEAD_DIST * math.sin(self.alpha)
        error = future_dist - desired_distance
        if self.DEBUG_MODE:
            print(a, b, self.alpha, self.current_dist, future_dist, error)
        return error


    def followRight(self,data, desired_distance):
        a = self.getRange(data, self.WALL_SCAN_ANGLE)
        b = self.getRange(data, 0.0)
        self.alpha = math.atan((a * math.cos(self.WALL_SCAN_ANGLE_RADIANS) - b) / (a * math.sin(self.WALL_SCAN_ANGLE_RADIANS)))
        self.current_dist = b * math.cos(self.alpha)
        future_dist = self.current_dist + self.LOOKAHEAD_DIST * math.sin(self.alpha)
        error = desired_distance - future_dist
        if self.DEBUG_MODE:
            print(a, b, self.alpha, self.current_dist, future_dist, error)
        return error

    def followCenter(self,data):
        error_dist_left = self.followLeft(data, 0.5)
        error_dist_right = self.followRight(data, 0.5)
        error_dist_center = error_dist_left + error_dist_right
        return error_dist_center


    def scan_callback(self,data):
        error = 0
        if self.OPERATION_MODE == 'left':
            error = self.followLeft(data, self.TARGET_DISTANCE_LEFT)  # left wall following
        elif self.OPERATION_MODE == 'right':
            error = self.followRight(data, self.TARGET_DISTANCE_RIGHT)  # right wall following
        elif self.OPERATION_MODE == 'center':
            error = self.followCenter(data)  # center line following
        msg = Float64()
        msg.data = error
        self.pid_pub.publish(msg)

        wall_data= wall_parameters()
        wall_data.wall_distance= self.current_dist
        wall_data.wall_angle = self.alpha
        wall_data.wall_follow_side = self.OPERATION_MODE
        self.wall_data_pub.publish(wall_data)


    def turning_callback(self,data):
        self.OPERATION_MODE = data.data


if __name__ == '__main__':
    try:
        pid_error = PIDerror()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
