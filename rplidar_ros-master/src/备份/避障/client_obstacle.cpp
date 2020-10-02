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
bool judge_front(float theta)
{
    if(fabs(theta)>1.57){return true;}
    else return false;
}
bool in_rectangle(float x,float y,float a,float b)
{
    if(fabs(x)<a&&fabs(y)<b){return true;}
    else return false; 

}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    int length = 0;
    float k1=0.7;
    float k2=0.55;
    float l1=0.62;
    float l2=0.6;  //K进行左右的控制，L进行前后的控制，1和2分别是前半周和后半周
    float t1=0.3; //隧道 的转向控制
    float project_x=0;
    float project_y=0;
    float deta_y1=0;
    float deta_y2=0;
    float safe_x1=0.45;
    float safe_x2=0.5;
    float safe_y1=0.8;
    float safe_y2=0.35;
    float tunnel_x1=0;
    float tunnel_x2=0;
    clock_t start,finish;
    float total_time;

    length=count;
    point point_array[length];
    
    //start=clock();
    for(int i = 0; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
        //float degree = RAD2DEG(rad);
        point_array[i].rho = fabs(scan->ranges[i]);
	point_array[i].theta = rad;

    }
    sort(point_array , point_array + length , cmp);
    project_x = point_array[0].rho * sin(point_array[0].theta);
    project_y = point_array[0].rho * cos(point_array[0].theta);
    
    if(judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.42,0.7))
    {
	ROS_INFO("FRONT");
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        deta_y1=safe_y1-project_y;
        if(project_x>0)
        {
	    laser_cmd.angular.z = k1 * (project_x-safe_x1) - l1 * deta_y1;
            ROS_INFO("turn left!!!!");
        }
	else if(project_x<0)//障碍物在左边
        {
            laser_cmd.angular.z = k1 * (safe_x1 + project_x) + l1 * deta_y1;
            ROS_INFO("turn right!!!!");
        }

    }
    else if(!judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.55,0.48))
    {
	ROS_INFO("BACK");
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        deta_y2=project_y-0.2;
        
        if(project_x<0)//ÕÏ°­ÎïÔÚ×ó±ß
        {
           laser_cmd.angular.z = k2 * project_x + l2 * deta_y2;
           ROS_INFO("turn left left!!!!");
        }
        else if(project_x>0)//ÕÏ°­ÎïÔÚÓÒ±ß
        {
           laser_cmd.angular.z = k2 * project_x - l2 * deta_y2;
           ROS_INFO("turn right right!!!!");
        }

    }
    else
    {
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = -1;
        ROS_INFO("camera control");
    }

    //finish=clock();
    //total_time=(float)(finish-start)/CLOCKS_PER_SEC;
    //ROS_INFO("TOTALTIME: %f",total_time);
    ROS_INFO("distance: %f",point_array[0].rho);
    ROS_INFO("angle: %f",point_array[0].theta);




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
