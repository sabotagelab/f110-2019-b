#!/usr/bin/env python

import rospy
import csv
import math
from geometry_msgs.msg import PoseStamped

class writeWaypoints:
    def __init__(self):
        self.pastX = 0
        self.pastY = 0
        rospy.init_node('create_waypoints_node', anonymous=True)
        rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, self.waypoint_recieved)
        self.file_path = rospy.get_param('~waypoint_filepath', '')
        if self.file_path == '':
            raise ValueError('No any file path for instruction file')

    def waypoint_recieved(self, data):
        euclidDistance = math.sqrt(pow((data.pose.position.y - self.pastY),2) + pow((data.pose.position.x - self.pastX),2))

        if self.pastX == 0 and self.pastY == 0:
            print(data.pose.position.x, data.pose.position.y, 0)

        elif (euclidDistance > 0.02) or (euclidDistance < 2.0):
            print(data.pose.position.x, data.pose.position.y, 0)

        else:
            print("threshold not reached")

        self.pastX = data.pose.position.x
        self.pastY = data.pose.position.y


    def writeToCSV(self,x,y):
        with open(self.file_path, mode='w') as waypoint_file:
            waypoint_writer = csv.writer(waypoint_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            waypoint_writer.writerow([x, y, 0])
            # waypoint_writer.writerow(['Erica Meyers', 'IT', 'March'])

if __name__ == '__main__':
    waypoint_creator = writeWaypoints()
    rospy.spin()