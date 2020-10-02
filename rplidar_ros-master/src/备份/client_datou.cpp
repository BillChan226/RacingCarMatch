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
    float safety_y = 0.35 + 0.30 + 0.05;
    float k = 1.5;
    float l = 0.5;
    float sum_Y=0;
    float av_Y=0;
    int judgePoint=0;
    for(int i = 0; i < count/6; i++) {
        length++;
    }
    for(int i = count*5/6; i < count; i++) {
        length++;
    }

    point point_array[length];

    int j = 0;
    for(int i = 0; i < count/6; i++) {
	float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array[i].rho = scan->ranges[i];
        point_array[i].theta = rad;
        j++;
    }

    for(int i = count*5/6; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
	//float degree = RAD2DEG(rad);
        point_array[j].rho = scan->ranges[i];
        point_array[j].theta = rad;
        j++;
    }


    sort(point_array , point_array + length , cmp);
    for(int i = 0; i < length; i++)
    {
        float project_x = point_array[i].rho * sin(point_array[i].theta);
        float project_y = point_array[i].rho * cos(point_array[i].theta);
	if(i==0)
        {
            sum_Y=project_y;
            av_Y=sum_Y;
            judgePoint=1;
        }
        if(fabs(project_y-av_Y)<0.05&&i>0)
        {
                judgePoint+=1;
                sum_Y=sum_Y+project_y;
                av_Y=sum_Y/(i+1);
        }


	}
    if(judgePoint>20)
    {
        ROS_INFO("shangpo");
    }
    else
    {
        ROS_INFO("pd");
    }

    /*float k = 1.0;
    laser_cmd.angular.z = k * (y_nearest_right + y_nearest_left);
    ROS_INFO("error: %f", y_nearest_right + y_nearest_left);
    ROS_INFO(" %f, %f", y_nearest_right, y_nearest_left);*/
    laser_cmd.angular.z = 0;
    //ROS_INFO("laser_cmd.angular.z: %f", laser_cmd.angular.z);
    pub.publish(laser_cmd);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    laser_cmd.linear.x = 0.0;

    ros::spin();

    return 0;
}
