#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <queue>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
std::queue<double> myQueue;
int queueSize = 10;
double sum = 0;
double movingAverage = -100000;
void chatterCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_INFO("I heard: [%lf]", msg->linear.x);
    myQueue.push(msg->linear.x);
    sum += msg->linear.x;

    if(myQueue.size() > queueSize){
        sum -= myQueue.front();
        myQueue.pop();
        movingAverage = sum / (double)queueSize;
        ROS_INFO("Moving Average: %lf", movingAverage);
    }
}

int main(int argc, char **argv)
{
    
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "av");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("average_velocity", 1000);
    ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 1000, chatterCallback);

    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */


    ros::Rate loop_rate(5);

    /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
    while (ros::ok()){
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
        std_msgs::Float32 msg;
        if (movingAverage != -100000){
            msg.data = (float)movingAverage;
            chatter_pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}