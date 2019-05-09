#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64, Float32
import math
import numpy as np

KP = 40.0
KD = 30.0
KI = 0.0
error_sum = 0
last_error = 0
target_vel = 0
DRIVE_VEL = 0
MAX_DELTA_STEERING_ANGLE = 30
INSTRUCTION_MODE = False

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
# Based on the error (data.data), determine the car's required velocity and steering angle.
def control_callback(data):
    global KP, KD, KI, error_sum, last_error, target_vel
    error = data.data
    output = KP * error + KI * error_sum + KD * (error - last_error)

    last_error = error
    if output > MAX_DELTA_STEERING_ANGLE:
        output = MAX_DELTA_STEERING_ANGLE
    elif output < -MAX_DELTA_STEERING_ANGLE:
        output = -MAX_DELTA_STEERING_ANGLE
    steering_angle = math.radians(output)
    if INSTRUCTION_MODE:
        target_vel = DRIVE_VEL
    else:
        alpha_degrees = abs(output)
        if 0 <= alpha_degrees <= 10:
            target_vel = 2.0
        elif 10 < alpha_degrees <= 20:
            target_vel = 1.5
        else:
            target_vel = 0.5

    msg = drive_param()
    msg.velocity = target_vel
    msg.angle = steering_angle
    # print("output=", output, "vel=", target_vel)
    pub.publish(msg)


def velocity_callback(data):
    global DRIVE_VEL, INSTRUCTION_MODE
    INSTRUCTION_MODE = True
    DRIVE_VEL = data.data


if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("pid_error", Float64, control_callback)
    rospy.Subscriber("drive_vel", Float32, velocity_callback)
    rospy.spin()
