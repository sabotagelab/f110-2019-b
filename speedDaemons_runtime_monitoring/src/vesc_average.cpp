
#include "ros/ros.h"
#include <queue>
#include "std_msgs/Float64.h"
#include <vesc_msgs/VescStateStamped.h>
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

 void vescVelCallback(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{
    ROS_INFO("I heard x speed=: [%f]", msg->state.speed);
    sliding_window.calc_average( msg->state.speed);
    ROS_INFO("Average speed=: [%f]",sliding_window.get_average());
    publish_flag=true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speedDaemons_average_vesc");
  ros::NodeHandle n;
  ros::Publisher avg_vel_pub = n.advertise<std_msgs::Float64>("average_velocity", 100);
  ros::Subscriber vel_sub = n.subscribe("sensors/core", 10, vescVelCallback);
  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    if(publish_flag)
    {
        std_msgs::Float64 avg_data;
        avg_data.data=sliding_window.get_average();
        avg_vel_pub.publish(avg_data);
        ROS_INFO("Average speed at 5 Hz rate=: [%f]", avg_data.data);
        publish_flag=false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


