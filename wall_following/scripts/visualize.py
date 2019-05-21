#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, String
import rospy

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher_wall_alignment = rospy.Publisher('/visualization_wall_alignment', Marker, queue_size="1")
publisher_turns = rospy.Publisher('/visualization_turns', Marker, queue_size="1")

class Visualizer:
    # default wall following operation mode is right
    wallAlignment = "following "+rospy.get_param('follow_wall', 'right')+" wall"
    turnStatus = "turnCompleted"

    def turn_status_callback(self,data):
        if data.data == "turnCompleted":
            self.turnStatus = data.data

    def turning_callback(self,data):
        self.wallAlignment = "following "+data.data+" wall"
        self.turnStatus = "Turning Now"

    def publish_text(self,text,x,y,publisher):
        marker = Marker()

        # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
        marker.header.frame_id = "/laser"

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0 # or set this to 0

        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.lifetime=rospy.Duration(0.1)

        marker.scale.x = 0.2 # If marker is too small in Rviz can make it bigger here
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the MarkerArray
        publisher.publish(marker)

if __name__ == '__main__':
    rospy.init_node('visualizer')
    visualizer = Visualizer()
    rate = rospy.Rate(200)
    rospy.Subscriber('turning_mode', String, visualizer.turning_callback)
    rospy.Subscriber("instruction_feedback", String, visualizer.turn_status_callback)

    while not rospy.is_shutdown():
        visualizer.publish_text(visualizer.wallAlignment,-1,1,publisher_wall_alignment)
        visualizer.publish_text(visualizer.turnStatus,0,1,publisher_turns)
        rate.sleep()
