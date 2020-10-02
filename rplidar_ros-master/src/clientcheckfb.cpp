#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <algorithm>

using namespace std;
#define RAD2DEG(x) ((x)*180./M_PI)

geometry_msgs::Twist laser_cmd;

ros::Subscriber sub;

ros::Publisher pub;

struct point                //极坐标
{
    float rho;
    float theta;
};

bool cmp(point x, point y)
{
    return x.rho < y.rho;
}
bool function1(float a,float b)
{
    if(a<0.5&&b<0.5)
    {
        return true;
    }
    else{return false;}
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
     int count = scan->scan_time / scan->time_increment;
     int length = count/2;
     float k1=1;
     float k2=0.25;
     float l1=1;
     float l2=0.5;  //K进行左右的控制，L进行前后的控制，1和2分别是前半周和后半周
     float project_x1=0;
     float project_y1=0;
     float project_x2=0;
     float project_y2=0;
     float deta_y1=0;
     float deta_y2=0;
     float safe_x1=0.45;
     float safe_x2=0.5;
     float safe_y1=0.8;
     float safe_y2=0.35;
     float tunnel_x1=0;
     float tunnel_x2=0;
     point point_array1[length];
     point point_array2[length];
     int j=0;
     int jj=0;
     for(int i = 0; i < count/4; i++) {
	float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array1[i].rho = scan->ranges[i];
	point_array1[i].theta = rad;
	j++;
    }

    for(int i = count*3/4; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array1[j].rho = scan->ranges[i];
	point_array1[j].theta = rad;
	j++;
    }
    for(int i = count*1/4;i < count*2/4; i++) {
	float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array2[i].rho = fabs(scan->ranges[i]);
	point_array2[i].theta = rad;
	jj++;
    }

    for(int i = count*2/4; i < count*3/4; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array2[jj].rho = fabs(scan->ranges[i]);
	point_array2[jj].theta = rad;
	jj++;
    }
     sort(point_array1 , point_array1 + length , cmp);
     sort(point_array2 , point_array2 + length , cmp);
     ROS_INFO("point theta1 theta: %f", point_array1[0].theta);
     ROS_INFO("point theta1 rho: %f", point_array1[0].rho);
     ROS_INFO("point theta2 theta: %f", point_array2[0].theta);
     ROS_INFO("point theta2 rho: %f", point_array2[0].rho);
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
