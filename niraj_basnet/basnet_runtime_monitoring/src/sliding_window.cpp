
#include "ros/ros.h"
#include <queue>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

using namespace std;

class SlidingWindow
{
private:
        float sum;
        int window;
        float mean;
        queue<float> data_queue;


public:
       SlidingWindow(int window=1)
       {
           sum=0;
           mean=0;
           this->window=window;
       }

       void setWindow(int window)
       {
           this->window=window;
       }

       void calc_average(float data)
       {
           data_queue.push(data);
           if(data_queue.size()<=window)
           {
               sum=sum+data;
               mean=sum/window;
           }
           else
           {
               sum=sum+data-data_queue.front();
               data_queue.pop();
               mean=sum/window;
           }
       }
       
       float get_average(void)
       {
           return mean;
       }
};

SlidingWindow sliding_window(10);
bool publish_flag=false;

 void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("I heard speed=: [%f]", msg->linear.x);
    sliding_window.calc_average( msg->linear.x);
    ROS_INFO("Average speed=: [%f]", sliding_window.get_average());
    publish_flag=true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basnet_average");
  ros::NodeHandle n;
  ros::Publisher avg_vel_pub = n.advertise<std_msgs::Float32>("average_velocity", 100);
  ros::Subscriber vel_sub = n.subscribe("/turtle1/cmd_vel", 100, cmdVelCallback);
  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if(publish_flag)
    {
        std_msgs::Float32 avg_data;
        avg_data.data=sliding_window.get_average();
        avg_vel_pub.publish(avg_data);
        ROS_INFO("Average speed at 5 Hz=: [%f]", avg_data.data);
        publish_flag=false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


