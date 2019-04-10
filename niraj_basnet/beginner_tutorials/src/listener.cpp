
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/robot_pose.h"
#include <sstream>

 void poseChatterCallback(const beginner_tutorials::robot_pose::ConstPtr& msg)
{
    ROS_INFO("I heard left speed: [%f]", msg->left_speed);
    ROS_INFO("I heard right speed: [%f]", msg->right_speed);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pose_chatter", 10, poseChatterCallback);
  ros::spin();
  return 0;
}
