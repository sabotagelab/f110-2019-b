#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
import pdb

pid_pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 5.0
MIN_ANGLE = 0
MAX_ANGLE = 180
FOV = MAX_ANGLE - MIN_ANGLE
LOOKAHEAD_DIST =0.2
WALL_SCAN_ANGLE = math.radians(60)
alpha = 0
TARGET_DISTANCE_RIGHT = 1.0
TARGET_DISTANCE_LEFT = 1.0
OPERATION_MODE = 'right' # left, right and center
DEBUG_MODE = False


# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
    range_idx = len(data.ranges) * (angle - MIN_ANGLE) / FOV
    range = data.ranges[int(range_idx)]
    if math.isinf(range):
        return data.range_max
    elif math.isnan(range):
        return getRange(data,angle-1)
    return range


# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
    global alpha, WALL_SCAN_ANGLE, LOOKAHEAD_DIST
    a = getRange(data, 120)
    b = getRange(data, 179)
    alpha = -math.atan((a * math.cos(WALL_SCAN_ANGLE) - b) / (a * math.sin(WALL_SCAN_ANGLE)))
    current_dist = b * math.cos(alpha)
    future_dist = current_dist - LOOKAHEAD_DIST * math.sin(alpha)
    error = future_dist - desired_distance
    if DEBUG_MODE:
        print(a, b, alpha, current_dist, future_dist, error)
    return error


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
    global alpha, WALL_SCAN_ANGLE, LOOKAHEAD_DIST
    a = getRange(data, 60)
    b = getRange(data, 0)
    alpha = math.atan((a * math.cos(WALL_SCAN_ANGLE) - b) / (a * math.sin(WALL_SCAN_ANGLE)))
    current_dist = b * math.cos(alpha)
    future_dist = current_dist + LOOKAHEAD_DIST * math.sin(alpha)
    error = desired_distance - future_dist
    if DEBUG_MODE:
        print(a, b, alpha, current_dist, future_dist, error)
    return error


# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
    global alpha
    error_dist_left = followLeft(data, 1.0)
    error_dist_right = followRight(data, 1.0)
    erro_dist_center = error_dist_left + error_dist_right
    return erro_dist_center


def scan_callback(data):
    error = 0
    if OPERATION_MODE == 'left':
        error = followLeft(data, TARGET_DISTANCE_LEFT)  # left wall following
    elif OPERATION_MODE == 'right':
        error = followRight(data, TARGET_DISTANCE_RIGHT)  # right wall following
    elif OPERATION_MODE == 'center':
        error = followCenter(data)  # center line following
    msg = Float64()
    msg.data = error
    pid_pub.publish(msg)


def turning_callback(data):
    global OPERATION_MODE
    OPERATION_MODE = data.data
    print(OPERATION_MODE)


if __name__ == '__main__':
    global OPERATION_MODE
    rospy.init_node('pid_error_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.Subscriber("turning_mode", String, turning_callback)
    OPERATION_MODE = rospy.get_param('follow_wall', 'right') # left, right and center
    print("op-mode=",OPERATION_MODE)

    rospy.spin()
