#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, Point, Vector3
from speedDaemons_gap_finding.msg import Gap, gaps
import math
import numpy as np
import copy


class GapFinding:
    def __init__(self):
        rospy.init_node('find_gap_node', anonymous=True)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.gap_center_pub = rospy.Publisher("gap_center", Vector3, queue_size=1)
        self.gaps_pub = rospy.Publisher("lidar_gaps", gaps, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.angle_max = math.radians(rospy.get_param('~angle_max', 90.0))
        self.angle_min = math.radians(rospy.get_param('~angle_min', -90.0))
        self.angle_initialize_flag = False
        self.lookahead_range = rospy.get_param('~lookahead_range', 3.0)
        self.cluster_density = 4.0
        self.skip_points = 3
        self.distance_tolerance = 0.1
        self.MIN_GAP_WIDTH = 1.0
        self.cluster = []
        self.gap_widths = []
        self.publish_flag = False

    def toXYZ(self, radius, theta):
        return Point(radius * math.cos(theta), radius * math.sin(theta), 0.0)

    def distance(self, point1, point2):
        p1 = np.array((point1.x, point1.y))
        p2 = np.array((point2.x, point2.y))
        return np.linalg.norm(p1 - p2)

    def scan_callback(self, data):
        self.find_gap(data)

    def find_gap(self, scan_data):
        angle_delta = scan_data.angle_increment
        range_data = scan_data.ranges
        last_angle = self.angle_min
        last_radius = 0
        current_angle = scan_data.angle_min
        start_range = 0
        start_angle = last_angle
        end_range = 0
        end_angle = 0
        cluster_count = 0
        gap_width = 0
        gap_start = False
        noise_reset = False
        temp_cluster = []
        temp_gapWidth = []
        self.angle_initialize_flag = False

        for dist in range_data:
            if self.angle_min <= current_angle <= self.angle_max:
                if scan_data.range_min <= dist and dist <= scan_data.range_max:
                    if not self.angle_initialize_flag:
                        last_radius = dist
                        start_range = dist
                        self.angle_initialize_flag = True
                    if dist <= self.lookahead_range:
                        if gap_start:
                            cluster_count = cluster_count + 1
                        gap_distance = self.distance(self.toXYZ(dist, current_angle),
                                                        self.toXYZ(last_radius, last_angle))
                        if gap_distance > self.distance_tolerance:
                            if not gap_start:
                                gap_start = True
                                end_range = dist
                                end_angle = current_angle
                                start_range = last_radius
                                start_angle = last_angle
                                gap_width = gap_distance
                            else:
                                cluster_count = 0
                                gap_start = False
                                noise_reset = True

                        if cluster_count >= self.cluster_density:
                            temp_cluster.append((start_range, start_angle, end_range, end_angle))
                            # print(start_range, start_angle, end_range, end_angle)
                            temp_gapWidth.append(gap_width)
                            cluster_count = 0
                            gap_start = False

                        if noise_reset:
                            last_angle = start_angle
                            last_radius = start_range
                            noise_reset = False
                        else:
                            last_angle = current_angle
                            last_radius = dist
            current_angle = current_angle + angle_delta

        if len(temp_gapWidth) > 0:
            max_index = int(np.argmax(np.array(temp_gapWidth)))
            if temp_gapWidth[max_index] > gapfinding.MIN_GAP_WIDTH:
                p1 = self.toXYZ(temp_cluster[max_index][0], temp_cluster[max_index][1])
                p2 = self.toXYZ(temp_cluster[max_index][2], temp_cluster[max_index][3])
                p3 = Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, 0)

                gap_center_point = Vector3()
                gap_center_point.x, gap_center_point.y, gap_center_point.z = p3.x, p3.y, p3.z
                self.gap_center_pub.publish(gap_center_point)
            # print(temp_cluster)
        self.cluster = temp_cluster
        self.gap_widths = temp_gapWidth
        self.publish_flag = True


if __name__ == '__main__':
    gapfinding = GapFinding()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if gapfinding.publish_flag:
            scan_gaps = gaps()
            scan_gaps.header.frame_id = '/laser'
            scan_gaps.header.stamp = rospy.Time.now()
            for index, data in enumerate(gapfinding.cluster):
                if gapfinding.gap_widths[index]> gapfinding.MIN_GAP_WIDTH:
                    single_gap = Gap()
                    single_gap.startRange = data[0]
                    single_gap.startAngle = data[1]
                    single_gap.endRange = data[2]
                    single_gap.endAngle = data[3]
                    single_gap.gapWidth = gapfinding.gap_widths[index]
                    scan_gaps.gapdata.append(single_gap)
            gapfinding.gaps_pub.publish(scan_gaps)
            gapfinding.publish_flag = False
        rate.sleep()
