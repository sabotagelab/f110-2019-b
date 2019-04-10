
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/robot_pose.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<beginner_tutorials::robot_pose>("pose_chatter", 10);
  ros::Rate loop_rate(10);
  beginner_tutorials::robot_pose pose_msg;
  while (ros::ok())
  {
    pose_msg.left_speed=20;
    pose_msg.right_speed=20;
    chatter_pub.publish(pose_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
