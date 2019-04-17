#!/usr/bin/env python

from sklearn.cluster import KMeans
import math
import rospy
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Float64, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from speedDaemons_kmeans_gap_finding.msg import Gap, gaps
import numpy as np
from visualization_msgs.msg import Marker

pub = rospy.Publisher('gap_center', Vector3, queue_size=1)
marker_publisher1 = rospy.Publisher('visualization_marker1', Marker, queue_size=1)
marker_publisher2 = rospy.Publisher('visualization_marker2', Marker, queue_size=1)
gaps_pub = rospy.Publisher("lidar_gaps", gaps, queue_size=1)


def showPoints(data, publisher, color):
    marker = Marker()

    # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/laser"
    # rospy.loginfo(rospy.get_caller_id() + "test")
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.x)
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = data.z  # or set this to 0

    marker.type = marker.SPHERE

    marker.scale.x = .1  # If marker is too small in Rviz can make it bigger here
    marker.scale.y = .1
    marker.scale.z = .1
    marker.color.a = 1.0
    marker.color.r = color.x
    marker.color.g = color.y
    marker.color.b = color.z
    publisher.publish(marker)


def scan_callback(data):
    # must convert from angle + distance to coordinates with x and y before can perform clustering
    angles = []
    now = rospy.Time.now()
    data.ranges = np.array(data.ranges)
    for c, i in enumerate(data.ranges):
        angles.append(data.angle_min + float(c) * data.angle_increment)
    angles = np.array(angles)
    x = data.ranges * np.cos(angles)
    y = data.ranges * np.sin(angles)
    coordinates = np.column_stack((x, y))
    coordinates = coordinates[x < 3.0]
    end_points = []
    pred = KMeans(n_clusters=5, n_init=2, random_state=2).fit_predict(coordinates)
    gap_widths = []
    last_pred_value = pred[0]
    for idx, data in enumerate(pred):
        if data != last_pred_value:
            p1 = np.array([coordinates[idx - 1][0], coordinates[idx - 1][1]])
            p2 = np.array([coordinates[idx][0], coordinates[idx][1]])
            end_points.append(coordinates[idx - 1])
            end_points.append(coordinates[idx])
            gap_widths.append(np.linalg.norm(p1 - p2))
            last_pred_value = data

    gap_widths = np.array(gap_widths)
    max_width_idx = np.argmax(gap_widths)
    max_width = gap_widths[max_width_idx]

    # print(max_width)
    # print(rospy.Time.now()-now)
    msg = Vector3()
    msg.x = (end_points[max_width_idx * 2][0] + end_points[max_width_idx * 2 + 1][0]) / 2.0
    msg.y = (end_points[max_width_idx * 2][1] + end_points[max_width_idx * 2 + 1][1]) / 2.0
    pub.publish(msg)

    scan_gaps = gaps()


    scan_gaps.header.frame_id = '/laser'
    scan_gaps.header.stamp = rospy.Time.now()
    for index in range(int(len(end_points) / 2)):
        single_gap = Gap()
        single_gap.startX = end_points[index * 2][0]
        single_gap.startY = end_points[index * 2][1]
        single_gap.endX = end_points[index * 2 + 1][1]
        single_gap.endY = end_points[index * 2 + 1][1]
        single_gap.gapWidth = gap_widths[index]
        scan_gaps.gapdata.append(single_gap)
    gaps_pub.publish(scan_gaps)

# msg1=Vector3(end_points[max_width_idx*2][0],end_points[max_width_idx*2][1],0)
# msg2=Vector3(end_points[max_width_idx*2+1][0],end_points[max_width_idx*2+1][1],0)
# showPoints(msg1,marker_publisher1,Point(0,1,0))
# showPoints(msg2,marker_publisher2,Point(0,1,1))

if __name__ == '__main__':
    rospy.init_node('find_gap_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
