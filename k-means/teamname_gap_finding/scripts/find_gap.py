#!/usr/bin/env python

from sklearn.cluster import KMeans

import math
import rospy
from geometry_msgs.msg import Vector3,Point
from std_msgs.msg import Float64, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
import numpy as np
from visualization_msgs.msg import Marker



pub = rospy.Publisher('gap_center', Vector3, queue_size=1)
marker_publisher1 = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
marker_publisher2 = rospy.Publisher('visualization_marker2', Marker, queue_size=10)
marker_publisher3 = rospy.Publisher('visualization_marker3', Marker, queue_size=10)
marker_publisher4 = rospy.Publisher('visualization_marker4', Marker, queue_size=10)
marker_publisher5 = rospy.Publisher('visualization_marker5', Marker, queue_size=10)
marker_publisher6 = rospy.Publisher('visualization_marker6', Marker, queue_size=10)
marker_publisher7 = rospy.Publisher('visualization_marker7', Marker, queue_size=10)
# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def show_in_rviz1(obstacle_points,color):
	obstacle_points_marker = Marker(
                   type=Marker.POINTS,
                   points=obstacle_points,
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='/laser'),
                   color=ColorRGBA(color.x, color.y, color.z, 0.8))

	marker_publisher1.publish(obstacle_points_marker)
def show_in_rviz2(obstacle_points,color):
	obstacle_points_marker = Marker(
                   type=Marker.POINTS,
                   points=obstacle_points,
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='/laser'),
                   color=ColorRGBA(color.x, color.y, color.z, 0.8))

	marker_publisher2.publish(obstacle_points_marker)
def show_in_rviz3(obstacle_points,color):
	obstacle_points_marker = Marker(
                   type=Marker.POINTS,

                   points=obstacle_points,
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='/laser'),
                   color=ColorRGBA(color.x, color.y, color.z, 0.8))

	marker_publisher3.publish(obstacle_points_marker)
def show_in_rviz4(obstacle_points,color):
	obstacle_points_marker = Marker(
                   type=Marker.POINTS,
                   id=0,
                   lifetime=rospy.Duration(0.1),
                   points=obstacle_points,
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='/laser'),
                   color=ColorRGBA(color.x, color.y, color.z, 0.8))

	marker_publisher4.publish(obstacle_points_marker)
def show_in_rviz5(obstacle_points,color):
	obstacle_points_marker = Marker(
                   type=Marker.POINTS,
            
                   points=obstacle_points,
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='/laser'),
                   color=ColorRGBA(color.x, color.y, color.z, 0.8))

	marker_publisher5.publish(obstacle_points_marker)

def showPoints(data,publisher,color):
    marker = Marker()

# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/laser"
    # rospy.loginfo(rospy.get_caller_id() + "test")
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.x)
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = data.z # or set this to 0

    marker.type = marker.SPHERE

    marker.scale.x = .1 # If marker is too small in Rviz can make it bigger here
    marker.scale.y = .1
    marker.scale.z = .1
    marker.color.a = 1.0
    marker.color.r = color.x
    marker.color.g = color.y
    marker.color.b = color.z

    # Publish the MarkerArray
    print("Sending marker")
    publisher.publish(marker)
def toPoints(cluster_data):
	points=[]
	for i in cluster_data:
		points.append(Point(i[0],i[1],0))
	return points

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
	
	np.random.seed(1000)

	coordinates = np.column_stack((x,y))
	coordinates = coordinates[x < 3.0]
	end_points=[]
	pred = KMeans(n_clusters=5,random_state=2).fit_predict(coordinates)
	gap_widths=[]
	last_pred_value=pred[0]
	for idx,data in enumerate(pred):
		if data!=last_pred_value:
			p1=np.array([coordinates[idx-1][0],coordinates[idx-1][1]])
			p2=np.array([coordinates[idx][0],coordinates[idx][1]])
			end_points.append(coordinates[idx-1])
			end_points.append(coordinates[idx])
			gap_widths.append(np.linalg.norm(p1-p2))
			print(p1,p2,)
			last_pred_value=data
	
	gap_widths=np.array(gap_widths)
	max_width_idx= np.argmax(gap_widths)
	max_width= gap_widths[max_width_idx]
	
	print(max_width)
	msg.x = (end_points[max_width_idx*2][0]+end_points[max_width_idx*2+1][0])/2.0
	msg.y = (end_points[max_width_idx*2][1]+end_points[max_width_idx*2+1][1])/2.0
	pub.publish(msg)

	# msg1=Vector3(end_points[max_width_idx*2][0],end_points[max_width_idx*2][1],0)
	# msg2=Vector3(end_points[max_width_idx*2+1][0],end_points[max_width_idx*2+1][1],0)
	# showPoints(msg1,marker_publisher6,Point(0,1,0))
	# showPoints(msg2,marker_publisher7,Point(0,1,1))

	# print(end_points)
	# clusters = [[] for _ in range(5)]
	# for i in range(0,len(coordinates)):
	# 	clusters[pred[i]].append(coordinates[i])
	# print(str(clusters[0][0][0]) + " " + str(clusters[0][0][1])+"\n" + str(clusters[0][-1][0]) + " " + str(clusters[0][-1][1]) + "\n")
	# print(str(clusters[1][0][0]) + " " + str(clusters[1][0][1])+"\n" + str(clusters[1][-1][0]) + " " + str(clusters[1][-1][1]) + "\n")
	# print(str(clusters[2][0][0]) + " " + str(clusters[2][0][1])+"\n" + str(clusters[2][-1][0]) + " " + str(clusters[2][-1][1]) + "\n")
	# print(str(clusters[3][0][0]) + " " + str(clusters[3][0][1])+"\n" + str(clusters[3][-1][0]) + " " + str(clusters[3][-1][1]) + "\n")
	# print(str(clusters[4][0][0]) + " " + str(clusters[4][0][1])+"\n" + str(clusters[4][-1][0]) + " " + str(clusters[4][-1][1]) + "\tend \n")
	

	# show_in_rviz1(toPoints(clusters[0]),Point(1,0,0))
	# show_in_rviz2(toPoints(clusters[1]),Point(0,1,0))
	# show_in_rviz3(toPoints(clusters[2]),Point(0,0,1))
	# show_in_rviz4(toPoints(clusters[3]),Point(1,1,0))
	# show_in_rviz5(toPoints(clusters[4]),Point(0,1,1))
	# msg.x = clusters[0][0][0]
	# msg.y = clusters[0][0][1]
	# pub.publish(msg)

	# msg.x = clusters[0][-1][0]
	# msg.y = clusters[0][-1][1]
	# pub.publish(msg)

	# coordinateTuple = [[] for _ in range(2)]
	# maxDistance = 0
	# for k in range(0,5):
	# 	for l in range(0,5):
	# 		if k != l:
	# 			for i in clusters[k]:
	# 				for j in clusters[l]:
	# 					if (math.sqrt(abs(i[0] - j[0]) + abs(i[1] - j[1]))) > maxDistance:
	# 						coordinateTuple[0] = i
	# 						coordinateTuple[1] = j
	# 						maxDistance = abs(i[0] - j[0]) + abs(i[1] - j[0])
	# # print(maxDistance)

	# if coordinateTuple[0][0] < coordinateTuple[1][0]:
	# 	goalX = (coordinateTuple[1][0] - coordinateTuple[0][0])/2 + coordinateTuple[0][0]		
	# else:
	# 	goalX = (coordinateTuple[0][0] - coordinateTuple[1][0])/2 + coordinateTuple[1][0]

	# if coordinateTuple[0][1] < coordinateTuple[1][1]:
	# 	goalY = (coordinateTuple[1][1] - coordinateTuple[0][1])/2 + coordinateTuple[0][1]		
	# else:
	# 	goalY = (coordinateTuple[0][1] - coordinateTuple[1][1])/2 + coordinateTuple[1][1]
	# print(goalX)
	# if (maxDistance >= 10):
	# 	msg.x = goalX
	# 	msg.y = goalY
	# 	pub.publish(msg)
	



# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('gap_finding_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

