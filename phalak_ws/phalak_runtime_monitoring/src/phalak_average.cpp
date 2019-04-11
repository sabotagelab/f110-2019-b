#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>

using namespace std;

class AvgVelocity{
	float vel_avg = 0;
	public:	
	void velocityCallback(const geometry_msgs::Twist& msg);
	void setAvg(float new_vel_avg);
	float getAvg();
};

void AvgVelocity::setAvg(float new_vel_avg){
	vel_avg = new_vel_avg;
}

float AvgVelocity::getAvg(){
     	return vel_avg;
}


void AvgVelocity::velocityCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO("I heard [%f]", msg.linear.x);
  double cur_avg = msg.linear.x;
  float new_avg = (float) cur_avg;
  this->setAvg(new_avg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "phalak_average");
  ros::NodeHandle n;
  AvgVelocity avgvel;
  ros::Subscriber vel_sub = n.subscribe("/turtle1/cmd_vel", 1000, &AvgVelocity::velocityCallback, &avgvel);
  ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("average_velocity", 1000);
  ros::Rate loop_rate(5);

  while(ros::ok()){
    float sum = 0;
    for(int i = 0; i < 10; i++)
    {
      ros::spinOnce();
      sum += avgvel.getAvg();
    }
    std_msgs::Float32 tenVelAverage;
    tenVelAverage.data = sum / 10.0;
    ROS_INFO("Average of 10 sliding windows: [%f]", tenVelAverage.data);
    vel_pub.publish(tenVelAverage);
    avgvel.setAvg(0);
    loop_rate.sleep();
  }
  return 0;
}
