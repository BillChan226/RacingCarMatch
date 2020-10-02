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
static int tunnel_num=0;
static int over_tunnel=0;

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
bool near_judge(float a,float b,float bound)
{
    if(fabs(fabs(a)-fabs(b))<bound)
    {
        return true;
    }
    else return false;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    int length = 0;
    float k1=20;
    float k2=25;
    float l1=12;
    float l2=25;  //K进行左右的控制，L进行前后的控制，1和2分别是前半周和后半周
    float t1=0.2; //隧道 的转向控制
    float project_x=0;
    float project_y=0;
    float deta_y1=0;
    float deta_y2=0;
    float safe_x1=0.45;
    float safe_x2=0.5;
    float safe_y1=0.9;
    float safe_y2=0.35;
    float tunnel_x1=0;
    float tunnel_x2=0;
    clock_t start,finish;
    float total_time;
    float nearest_left=-10;
    float nearest_right=10;
    length=count;
    point point_array[length];

    //start=clock();
    for(int i = 0; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
        float x = scan->ranges[i] * sin(rad);
        //ROS_INFO("%f, %f", degree, scan->ranges[i]);
        //ROS_INFO("%d:", count);
        float y = scan->ranges[i] * cos(rad);
        //float degree = RAD2DEG(rad);
        point_array[i].rho = fabs(scan->ranges[i]);
        point_array[i].theta = rad;
        if(i>count/8&&i<count/4)
        {
            if(x>nearest_left)
            {
                nearest_left=x;
		
                
            }
        }
        if(i>count*3/4&&i<count*7/8)
        {
            if(x<nearest_right)
            {
                nearest_right=x;
		
            }
        }

    }
    sort(point_array , point_array + length , cmp);
    project_x = point_array[0].rho * sin(point_array[0].theta);
    project_y = point_array[0].rho * cos(point_array[0].theta);
    ROS_INFO("projectx: %f",project_x);
    ROS_INFO("projecty: %f",project_y);
    if(near_judge(nearest_left,nearest_right,0.1))
    {
	
	laser_cmd.linear.z = 1.0;
        ROS_INFO("tunnel control");
        float k = 4.0;
        laser_cmd.angular.z = k * (nearest_left + nearest_right);
    	tunnel_num+=1;
	

    }

    //else if(judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.42,0.9))
    else if(project_y<0&&project_y>-0.9&&fabs(project_x)<0.45)
    {
	ROS_INFO("FRONT");
        ROS_INFO("projectXXXX: %f",project_x);
        ROS_INFO("projectYYYY: %f",project_y);
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        deta_y1=safe_y1+project_y;

        if(project_x>-0.1)
        {
	    laser_cmd.angular.z = k1 * (project_x-safe_x1) - l1 * deta_y1;
            ROS_INFO("turn left!!!!");
        }
	else if(project_x<0)//
        {
            laser_cmd.angular.z = k1 * (safe_x1 + project_x) + l1 * deta_y1;
            ROS_INFO("turn right!!!!");
        }

    }
    //else if(!judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.55,0.01))
    else if(project_y>0&&project_y<0.24&&fabs(project_x)<0.55)
    {
        ROS_INFO("BACK");
        ROS_INFO("x: %f",project_x);
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        deta_y2=project_y-0.2;

        if(project_x<0)//obstacle is on the left
        {
           laser_cmd.angular.z = k2 * project_x + l2 * deta_y2;
           ROS_INFO("turn left left!!!!");
        }
        else if(project_x>0)
        {
           laser_cmd.angular.z = k2 * project_x - l2 * deta_y2;
           ROS_INFO("turn right right!!!!");
        }

    }
    else
    {
        
        laser_cmd.linear.z = -1;
        ROS_INFO("camera control");
        if (tunnel_num>400) {over_tunnel=1;}
    }

    //finish=clock();
    //total_time=(float)(finish-start)/CLOCKS_PER_SEC;
    //ROS_INFO("TOTALTIME: %f",total_time);
    


    bool obstacle_width[20] = {false};
    int index = 0;
    float nearest_y = -0.9;
    int current;
    for(int i = count*9/10; i <= count*11/10; ++i)
    {
        current = i%count;
        float rad = scan->angle_min + scan->angle_increment * current;
        float y = scan->ranges[current] * cos(rad);
        float x = scan->ranges[current] * sin(rad);
        if (obstacle_width[index] == false)
        {
            if (y > -0.45)//-0.9)
            {
                if (x < 0.25 - index * 0.025 && x > 0.25 - (index + 1) * 0.025)
                    obstacle_width[index] = true;
                if (y > nearest_y)
                    nearest_y = x;
            }
        }
        if (obstacle_width[index] == true || (x < 0.25 - (index + 1) * 0.025 && y > -0.9))
            ++index;
        if (index == 20)
            break;
    }

    int width_count = 0;
    for(int i = 0; i < 20; ++i)
    {
         width_count += obstacle_width[i];
    }

    if (width_count > 2 && width_count < 15&& over_tunnel==1)
    {
         laser_cmd.linear.x = 0.0;
         ROS_INFO("pedestrian______________________");
    }
    else if (width_count <= 2)
         laser_cmd.linear.x = 0.1;
    else
         ROS_INFO("______________________");


    if (width_count >= 2)
       laser_cmd.linear.x = 0.0;
    else
       laser_cmd.linear.x = 0.1;




    ROS_INFO("laser_cmd.angular.z: %f", laser_cmd.angular.z);
    pub.publish(laser_cmd);


}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    pub = n.advertise<geometry_msgs::Twist>("laser_cmd", 1);//cmd_vel

    //laser_cmd.linear.x = 0.1;

    ros::spin();

    return 0;
}
