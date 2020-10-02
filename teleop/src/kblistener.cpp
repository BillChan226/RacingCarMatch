#include "std_msgs/String.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("speed :  [%2f]", msg->linear.x);
    ROS_INFO("turn :   [%2f]", msg->angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, chatterCallback);
    ros::spin();
    return 0;
}
