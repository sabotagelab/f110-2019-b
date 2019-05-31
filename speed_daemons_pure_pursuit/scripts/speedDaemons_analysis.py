#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from wall_following.msg import pid_error_analysis


class WallFollowAnalysis:

    def __init__(self):
        rospy.init_node('speedDaemons_analysis', anonymous=True)
        self.error_pub = rospy.Publisher('wall_following_analysis', pid_error_analysis, queue_size=1)
        rospy.Subscriber("pid_error", Float64, self.error_callback)
        self.running_average = 0.0
        self.running_max = 0.0
        self.counter = 1.0

    def error_callback(self,msg):
        error= abs(msg.data)
        self.running_average = (self.running_average + error)/self.counter
        self.counter = self.counter + 1
        self.running_max = max(self.running_max,error)
        error_msg = pid_error_analysis()
        error_msg.average=self.running_average
        error_msg.max=self.running_max
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        error_analysis = WallFollowAnalysis()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


