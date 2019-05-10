#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Float32
from race.msg import drive_param
from speedDaemons_gap_finding.msg import Gap, gaps
from wall_following.msg import turn_instruction
import math


class Master:
    def __init__(self):
        rospy.init_node('master_node', anonymous=True)
        rospy.Subscriber('drive_instruction', turn_instruction, self.instruction_callback)
        rospy.Subscriber("lidar_gaps", gaps, self.turning_callback)
        self.turn_pub = rospy.Publisher('turning_mode', String, queue_size=10)
        self.vel_pub = rospy.Publisher('drive_vel', Float32, queue_size=10)
        self.feedback_pub = rospy.Publisher('instruction_feedback', String, queue_size=1)

        self.current_turn = ''
        self.current_vel = 0
        self.wall_follow_angle = 0
        self.wall_follow_vel = 0
        self.time_up = True
        self.DELTA_ANGLE = math.radians(45)
        self.STATUS_TURN_COMPLETE = 'turnCompleted'
        self.DEFAULT_VEL = 2

    def instruction_callback(self, msg):
        self.current_turn = msg.command
        self.current_vel = msg.velocity
        if self.current_turn == 'stop':
            vel = Float32()
            vel.data = self.current_vel
            self.vel_pub.publish(vel)
            self.feedback_pub.publish(self.STATUS_TURN_COMPLETE)

    def turning_mode(self, data):
        print(data)
        self.turn_pub.publish(data)
        vel = Float32()
        vel.data = self.current_vel
        self.vel_pub.publish(vel)

    def timerCallback(self, event):
        if not self.time_up:
            self.time_up = True
            self.feedback_pub.publish(self.STATUS_TURN_COMPLETE)
            vel = Float32()
            vel.data = self.DEFAULT_VEL
            self.vel_pub.publish(vel)

    def turning_callback(self, data):
        if self.current_turn != 'stop':
            if len(data.gapdata) > 1:
                for i in data.gapdata:
                    gap_mid_angle = (i.startAngle + i.endAngle) / 2.0
                    if gap_mid_angle > self.DELTA_ANGLE and self.current_turn == "left" and self.time_up:
                        # left_turn
                        self.turning_mode("left")
                        self.time_up = False
                        rospy.Timer(rospy.Duration(3), self.timerCallback, oneshot=True)

                    elif gap_mid_angle < -self.DELTA_ANGLE and self.current_turn == "right" and self.time_up:
                        # right_turn
                        self.turning_mode("right")
                        self.time_up = False
                        rospy.Timer(rospy.Duration(3), self.timerCallback, oneshot=True)

                    elif abs(gap_mid_angle) < math.degrees(20) and self.current_turn == "straight" and self.time_up:
                        # go_straight
                        self.turning_mode("straight")
                        rospy.Timer(rospy.Duration(3), self.timerCallback, oneshot=True)
                        self.time_up = False


if __name__ == '__main__':
    try:
        master = Master()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
