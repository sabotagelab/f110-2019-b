#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import math
import numpy as np

KP = 50.0
KD = 0.0
KI = 0.0
error_sum = 0
last_error = 0
MAX_DELTA_STEERING_ANGLE = 30

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


# Callback for receiving gap center data on the /gap_center topic
# data: the PID error from pid_error_node, published as a Float64
# Based on the error (data.data), determine the car's required velocity and steering angle.
def gap_callback(msg):
    global KP, KD, KI, error_sum, last_error
    error = msg.y
    output = KP * error + KI * error_sum + KD * (error - last_error)

    last_error = error
    if output > MAX_DELTA_STEERING_ANGLE:
        output = MAX_DELTA_STEERING_ANGLE
    elif output < -MAX_DELTA_STEERING_ANGLE:
        output = -MAX_DELTA_STEERING_ANGLE
    steering_angle = math.radians(output)

    alpha_degrees = abs(output)
    target_vel = 0
    if 0 <= alpha_degrees <= 10:
        target_vel = 1.0
    elif 10 < alpha_degrees <= 20:
        target_vel = 0.75
    else:
        target_vel = 0.5

    msg = drive_param()
    msg.velocity = target_vel
    msg.angle = steering_angle
    print("output=", output, "vel=", target_vel)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('gap_pid_node', anonymous=True)
    rospy.Subscriber("gap_center", Vector3, gap_callback)
    rospy.spin()
