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
     int length = 0;
     length=count/2;
     float k1=1;
     float k2=0.25;
     float l1=1;
     float l2=0.5;  //K进行左右的控制，L进行前后的控制，1和2分别是前半周和后半周
     float t1=0.2; //隧道 的转向控制
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

     for(int i=0;i<count;i++)
     {
         float rad = scan->angle_min + scan->angle_increment * i;
         if(fabs(rad)<1.57&&fabs(rad)>0.7)
         {
             point_array2[jj].rho = scan->ranges[i];
             point_array2[jj].theta = rad;
             jj++;
         }
         else
         {
             point_array1[j].rho = scan->ranges[i];
             point_array1[j].theta = rad;
             /*
             if(fabs(rad-2.355)<0.05)
             {
                 tunnel_x1=scan->ranges[i]*sin(rad);
                 
             }
             if(fabs(rad+2.355)<0.05)
             {
                 tunnel_x2=scan->ranges[i]*sin(rad);
                 
             }*/
             j++;
         }
     }
     sort(point_array1 , point_array1 + length , cmp);
     sort(point_array2 , point_array2 + length , cmp);
     ROS_INFO("1111111111111");
     project_x1 = point_array1[0].rho * sin(point_array1[0].theta);
     project_y1 = point_array1[0].rho * cos(point_array1[0].theta);
     project_x2 = point_array2[0].rho * sin(point_array2[0].theta);
     project_y2 = point_array2[0].rho * cos(point_array2[0].theta);
     //记得先看一下这个的正负是不是都是正确的
     if(fabs(project_x1)>safe_x1&&fabs(project_x2)>safe_x2&&fabs(project_y1)>safe_y1&&fabs(project_y2)>safe_y2)
     {
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = -1;
        ROS_INFO("camera control");
     }
     else if(function1(tunnel_x1,tunnel_x2))
     {
         ROS_INFO("tunnel:");
         laser_cmd.linear.x = 0.1;
         laser_cmd.angular.z = t1*(tunnel_x1+tunnel_x2);

     }


     else //避障
     {
         if(fabs(point_array1[0].rho)<fabs(point_array1[0].rho))//最近的障碍物在前面
         {
             if(point_array1[0].rho<0.5&&fabs(point_array1[0].theta)>2.9)
             {
                 laser_cmd.linear.x = 0.1;
                 laser_cmd.linear.z = 1;
                 laser_cmd.angular.z = 0;
                 ROS_INFO("pedestrian!!!!");
             }
             else
             {
                laser_cmd.linear.x = 0.1;
                laser_cmd.linear.z = 1;
                deta_y1=safe_y1-project_y1;
                if(project_x1<0)//障碍物在左边
                {
                    laser_cmd.angular.z = k1 * (safe_x1 + project_x1) + l1 * deta_y1;
                    ROS_INFO("turn right!!!!");
                }
                else if(project_x1>0)//障碍物在右边
                {
                    laser_cmd.angular.z = k1 * (project_x1-safe_x1) - l1 * deta_y1;
                    ROS_INFO("turn left!!!!");
                }
             }

         }
         else //最近的障碍物在后面
         {
             laser_cmd.linear.x = 0.1;
             laser_cmd.linear.z = 1;
             deta_y2=project_y2-0.2;
                //看一下现在是不是一定是负的，deta
             if(project_x2<0)//障碍物在左边
             {
                 laser_cmd.angular.z = k2 * project_x2 + l2 * deta_y2;
                 ROS_INFO("turn left left!!!!");
             }
             else if(project_x2>0)//障碍物在右边
             {
                 laser_cmd.angular.z = k2 * project_x2 - l2 * deta_y2;
                 ROS_INFO("turn right right!!!!");
             }



         }
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
