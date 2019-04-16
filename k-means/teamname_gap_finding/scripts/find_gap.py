#!/usr/bin/env python

from sklearn.cluster import KMeans

import math
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('gap_center', Vector3, queue_size=1)

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
	msg = Vector3()

	# must convert from angle + distance to coordinates with x and y before can perform clustering
	angles = []

	data.ranges = np.array(data.ranges)
	for c, i in enumerate(data.ranges):
		angles.append(data.angle_min + float(c)*data.angle_increment)

	angles = np.array(angles)

	x = data.ranges*np.cos(angles)
	y = data.ranges*np.sin(angles)

	# for c, i in enumerate(y):
	# 	rospy.loginfo(rospy.get_caller_id() + "y %f", y[c])
	# 	rospy.loginfo(rospy.get_caller_id() + "x %f", x[c])
	
	coordinates = np.column_stack((x,y))
	pred = KMeans(n_clusters=5).fit_predict(coordinates)
	# print(pred[0])
	clusters = [[] for _ in range(5)]
	for i in range(0,1080):
		clusters[pred[i]].append(coordinates[i])
	# print(clusters)

	coordinateTuple = [[] for _ in range(2)]
	maxDistance = 0
	for k in range(0,5):
		for l in range(0,5):
			if k != l:
				for i in clusters[k]:
					for j in clusters[l]:
						if (math.sqrt(abs(i[0] - j[0]) + abs(i[1] - j[1]))) > maxDistance:
							coordinateTuple[0] = i
							coordinateTuple[1] = j
							maxDistance = abs(i[0] - j[0]) + abs(i[1] - j[0])
	print(maxDistance)

	if coordinateTuple[0][0] < coordinateTuple[1][0]:
		goalX = (coordinateTuple[1][0] - coordinateTuple[0][0])/2 + coordinateTuple[0][0]		
	else:
		goalX = (coordinateTuple[0][0] - coordinateTuple[1][0])/2 + coordinateTuple[1][0]

	if coordinateTuple[0][1] < coordinateTuple[1][1]:
		goalY = (coordinateTuple[1][1] - coordinateTuple[0][1])/2 + coordinateTuple[0][1]		
	else:
		goalY = (coordinateTuple[0][1] - coordinateTuple[1][1])/2 + coordinateTuple[1][1]
	# print(goalX)
	if (maxDistance >= 10):
		msg.x = goalX
		msg.y = goalY
		pub.publish(msg)
	



# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('gap_finding_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

