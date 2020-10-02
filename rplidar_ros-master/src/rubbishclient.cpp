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

float previous_turn[40]={0.0};
static int num_turn=0;
bool cmp(point x, point y)
{
    return x.rho < y.rho;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    int length = 0;
    float safety = 0.27 + 0.05;    //0.27半车宽，0.15,安全距离
    float safety_y = 0.35 + 0.25;
    float k=1.5;
    float l=0.5;

    length=count/2;
    point point_array1[length];
    point point_array2[length];
    int j = 0;
    int jj = 0;
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
    for(int i = count/4; i < count/2; i++) {
	float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array2[i].rho = scan->ranges[i];
	    point_array2[i].theta = rad;
	    jj++;
    }

    for(int i = count/2; i < count*3/4; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array2[j].rho = scan->ranges[i];
	    point_array2[j].theta = rad;
	    jj++;
    }
    sort(point_array1 , point_array1 + length , cmp);
    sort(point_array2 , point_array2 + length , cmp);
    float project_x1 = point_array1[0].rho * sin(point_array1[0].theta);
    float project_y1 = point_array1[0].rho * cos(point_array1[0].theta);
    float project_x2 = point_array2[0].rho * sin(point_array2[0].theta);
    float project_y2 = point_array2[0].rho * cos(point_array2[0].theta);
    float deta_y1=safety_y-project_y1;
    float deta_y2=safety_y-project_y2;
    if(point_array1[0].rho<point_array2[0].rho)//the nearest obstacle is in front
    {
        if(fabs(project_x1)<0.35||fabs(project_y1)<0.75)
        {
            laser_cmd.linear.x = 0.1;
            laser_cmd.linear.z = -1;
            ROS_INFO("camera control");

        }
        else if(project_x1>0)//obstacle is on the right
        {
            ROS_INFO("turn left!");
            laser_cmd.angular.z = k * (project_x1 - safety) - fabs(l * deta_y1);
            previous_turn[num_turn]=laser_cmd.angular.z;
            num_turn+=1;
        }
        else if(project_x1<0)
        {
            ROS_INFO("turn right");
            laser_cmd.angular.z = k * (safety + project_x) + fabs(l * deta_y);
            previous_turn[num_turn]=laser_cmd.angular.z;
            num_turn+=1;
        }
        else
        {
            ROS_INFO("wrong front!!");
        }

    }
    else//release the previous process
    {
        ROS_INFO("release!!!!");
        laser_cmd.angular.z=-previous_turn[num_turn];
        num_turn-=1;
    }

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
