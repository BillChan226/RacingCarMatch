 /*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 */


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


bool turn_right_over = false;
bool turn_left_over = false;

struct point                //极坐标
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
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    int length = 0;    //扫描数组长度
    float safety = 0.27 + 0.05;    //0.27半车宽，0.15,安全距离
    float safety_y = 0.35 + 0.25;
    float k = 1.5;
    float l = 0.5;
    static int num_right=0;
    static int num_left=0;
    int mode=0;
    //bool exist_last_turn=false;
   
    for(int i = 0; i < count/4; i++) {
        length++;
    }
    for(int i = count*3/4; i < count; i++) {
        length++;
    }

    point point_array[length];

    int j = 0;
    for(int i = 0; i < count/4; i++) {
	float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array[i].rho = scan->ranges[i];
	    point_array[i].theta = rad;
	    j++;
    }

    for(int i = count*3/4; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array[j].rho = scan->ranges[i];
	    point_array[j].theta = rad;
	    j++;
    }

    sort(point_array , point_array + length , cmp);    //按距离从小到大排列


    float project_x = point_array[0].rho * sin(point_array[0].theta);
    float project_y = point_array[0].rho * cos(point_array[0].theta);
    float deta_y;

    if(fabs(project_x)>0.36||fabs(project_y)>0.6)
    {
        mode = 0;
    }
    else
    {
        deta_y=safety_y-project_y;//记得上一个if条件和safety_y要保持一致
        if(turn_right_over==false&&turn_left_over==false)//通过第一个障碍物，允许执行30次第一个左转或者右转
        {
            mode=1;
	    ROS_INFO("MODE 1");
        }
        else if(turn_right_over)//通过的障碍物不是第一个障碍，之前是右转
        {
            mode=2;
            ROS_INFO("MODE 2");
        }
        else if(turn_left_over)//通过的障碍物不是第一个障碍，之前是左转
        {
            mode=3;
            ROS_INFO("MODE 3");
        }



    }
    switch(mode)
    {
    case 0:
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = -1;
        ROS_INFO("camera control");
        break;

    case 1://上一次是右转
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        if(project_x<0)//障碍物在左边
        {
            laser_cmd.angular.z = k * (safety + project_x) + fabs(l * deta_y);
            num_right+=1;
            if(num_right>30)
            {
                turn_right_over=true;
                turn_left_over=false;
                 num_right=0;
                num_left=0;
            }
        }

        else if(project_x>0)
        {
            laser_cmd.angular.z = k * (project_x - safety) - fabs(l * deta_y);
            num_left+=1;
            if(num_left>30)
            {
                turn_right_over=false;
                turn_left_over=true;
                 num_right=0;
                num_left=0;
            }
        }
        else
        {
           ROS_INFO("control fail 1");
        }
        break;
    case 2://上一次是右转
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        if(project_x>0-0.2)
        {
            laser_cmd.angular.z = -0.5;//k * (project_x - safety-0.2) - fabs(l * deta_y);
            num_left+=1;
            if(num_left>30)
            {
                turn_right_over=false;
                turn_left_over=true;
                num_right=0;
                num_left=0;
            }
        }
        else
        {
           ROS_INFO("control fail 2");
        }

        break;
    case 3://上一次是左转
        laser_cmd.linear.x = 0.1;
        laser_cmd.linear.z = 1;
        if(project_x<0+0.2)//障碍物在左边
        {
            laser_cmd.angular.z = 0.5;//k * (safety + project_x) + fabs(l * deta_y);
            num_right+=1;
            if(num_right>30)
            {
                turn_right_over=true;
                turn_left_over=false;
                 num_right=0;
                num_left=0;
            }
        }

        /*else if(project_x>0)
        {
            laser_cmd.angular.z = k * (project_x - safety) - fabs(l * deta_y);
            num_left+=1;
            if(num_left>30)
            {
                turn_right_over=false;
                turn_left_over=true;
                num_right=0；
                num_left=0；
            }
        }*/
        else
        {
           ROS_INFO("control fail 3");
        }
        break;


    default:
        {
            ROS_INFO("do nothing");
        }
    }
    ROS_INFO("NUMLEFT: %d",num_left);
    ROS_INFO("NUMRIGHT: %d",num_right);
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
