#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <time.h>
#include <algorithm>
using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

geometry_msgs::Twist laser_cmd;

ros::Subscriber sub;

ros::Publisher pub;
struct point                //Œ«×ø±ê
{
    float rho;
    float theta;
};
bool cmp(point x, point y)
{
    return x.rho < y.rho;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    int length = 0;
    clock_t start,finish;
    float total_time;

    length=count;
    point point_array1[length];
    point point_array2[length/2];

    start=clock();
    for(int i = 0; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
        //float degree = RAD2DEG(rad);
        point_array1[i].rho = fabs(scan->ranges[i]);
	    point_array1[i].theta = rad;

    }

    
    sort(point_array1 , point_array1 + length , cmp);
    finish=clock();
    total_time=(float)(finish-start)/CLOCKS_PER_SEC;
    ROS_INFO("TOTALTIME: %f",total_time);
    ROS_INFO("distance: %f",point_array1[0].rho);
    ROS_INFO("angle: %f",point_array1[0].theta);




    ROS_INFO("laser_cmd.angular.z: %f", laser_cmd.angular.z);
    pub.publish(laser_cmd);
    

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //laser_cmd.linear.x = 0.1;

    ros::spin();

    return 0;
}
