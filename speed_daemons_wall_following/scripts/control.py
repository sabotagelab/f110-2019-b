#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64, Float32
import math
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
        self.DRIVE_VEL = 0
        self.KP = rospy.get_param('wall_Kp', 50.0)
        self.KD = rospy.get_param('wall_Kd', 30.0)
        self.KI = rospy.get_param('wall_Ki', 0.0)
        self.NORMAL_VEL = rospy.get_param('default_velocity', 2.0)
        self.NORMAL_TURNING_VEL = rospy.get_param('default_turning_velocity', 0.75)
        self.MAX_DELTA_STEERING_ANGLE = rospy.get_param('max_steering_angle', 30.0)
        self.INTEGRAL_LIMIT = rospy.get_param('integral_sum_limit', 10.0)
        self.INSTRUCTION_MODE = False
        print(self.KP,self.NORMAL_VEL)

    def constrain(self,value,min,max):
        if value>max:
            value=max
        elif value<min:
            value=min
        return value

    # Callback for receiving PID error data on the /pid_error topic
    # data: the PID error from pid_error_node, published as a Float64
    # Based on the error (data.data), determine the car's required velocity and steering angle.
    def control_callback(self,data):
        error = data.data
        self.error_sum = self.error_sum + error
        self.error_sum = self.constrain(self.error_sum,-self.INTEGRAL_LIMIT,self.INTEGRAL_LIMIT)
        output = self.KP * error + self.KI * self.error_sum + self.KD * (error - self.last_error)
        self.last_error = error
        output = self.constrain(output,-self.MAX_DELTA_STEERING_ANGLE, self.MAX_DELTA_STEERING_ANGLE)
        steering_angle = math.radians(output)
        if self.INSTRUCTION_MODE:
            self.target_vel = self.DRIVE_VEL
        else:
            alpha_degrees = abs(output)
            if 0 <= alpha_degrees <= 10:
                self.target_vel = self.NORMAL_VEL
            elif 10 < alpha_degrees <= 20:
                self.target_vel = (self.NORMAL_VEL+self.NORMAL_TURNING_VEL)/2.5
            else:
                self.target_vel = self.NORMAL_TURNING_VEL

        msg = drive_param()
        msg.velocity = self.target_vel
        msg.angle = steering_angle
        # print("output=", output, "vel=", target_vel)
        self.vel_pub.publish(msg)


    def velocity_callback(self,data):
        self.INSTRUCTION_MODE = True
        if data.data != -1:
            self.DRIVE_VEL = data.data
        elif data.data == -1:
            self.INSTRUCTION_MODE = False


if __name__ == '__main__':
    try:
        pid_control = PIDcontrol()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
