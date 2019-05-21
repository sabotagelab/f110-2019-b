#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64, Float32
import math
import copy
import numpy as np


class PIDcontrol:
    def __init__(self):
        rospy.init_node('pid_controller_node', anonymous=True)
        self.vel_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        rospy.Subscriber("pid_error", Float64, self.control_callback)
        rospy.Subscriber("drive_vel", Float32, self.velocity_callback)
        self.error_sum = 0
        self.last_error = 0
        self.target_vel = 0
        self.previous_vel = 0
        self.KP = rospy.get_param('wall_Kp', 50.0)
        self.KD = rospy.get_param('wall_Kd', 30.0)
        self.KI = rospy.get_param('wall_Ki', 0.0)
        self.DRIVE_VEL = rospy.get_param('default_velocity', 2.0)
        self.NORMAL_TURNING_VEL = rospy.get_param('default_turning_velocity', 0.75)
        self.MAX_DELTA_STEERING_ANGLE = rospy.get_param('max_steering_angle', 30.0)
        self.INTEGRAL_LIMIT = rospy.get_param('integral_sum_limit', 10.0)
        self.pid_frequency = rospy.get_param('pid_frequency', 10.0)
        self.acceleration = rospy.get_param('acceleration', 0.5)
        self.RAMPING_ENABLE_FLAG = rospy.get_param('ramping_enable', False)

    def constrain(self, value, min, max):
        if value > max:
            value = max
        elif value < min:
            value = min
        return value

    def ramp_velocity(self, target_vel, previous_vel, ramp_rate):
        sign = 1 if target_vel >= previous_vel else -1
        step_size = ramp_rate / self.pid_frequency
        delta = math.fabs(target_vel - previous_vel)
        if delta >= step_size:
            command_vel = previous_vel + sign * step_size
        else:
            command_vel = target_vel
        return command_vel

    # Callback for receiving PID error data on the /pid_error topic
    # data: the PID error from pid_error_node, published as a Float64
    # Based on the error (data.data), determine the car's required velocity and steering angle.
    def control_callback(self, data):
        error = data.data
        self.error_sum = self.error_sum + error
        self.error_sum = self.constrain(self.error_sum, -self.INTEGRAL_LIMIT, self.INTEGRAL_LIMIT)
        output = self.KP * error + self.KI * self.error_sum + self.KD * (error - self.last_error)
        self.last_error = error
        output = self.constrain(output, -self.MAX_DELTA_STEERING_ANGLE, self.MAX_DELTA_STEERING_ANGLE)
        steering_angle = math.radians(output)
        self.target_vel = self.DRIVE_VEL
        alpha_degrees = abs(output)
        if 0 <= alpha_degrees <= 10:
            self.target_vel = self.DRIVE_VEL
        elif 10 < alpha_degrees <= 20:
            self.target_vel = (self.DRIVE_VEL + self.NORMAL_TURNING_VEL) / 2.5
        else:
            self.target_vel = self.NORMAL_TURNING_VEL

        #ramping velocity based on acceleration value
        if self.RAMPING_ENABLE_FLAG:
            self.target_vel = self.ramp_velocity(self.target_vel,self.previous_vel,self.acceleration)
            self.previous_vel = copy.deepcopy(self.target_vel)

        msg = drive_param()
        msg.velocity = self.target_vel
        msg.angle = steering_angle
        # print("output=", output, "vel=", target_vel)
        self.vel_pub.publish(msg)

    def velocity_callback(self, data):
        self.DRIVE_VEL = data.data


if __name__ == '__main__':
    try:
        pid_control = PIDcontrol()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
