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


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define PI 3.1415926

geometry_msgs::Twist vel_msg;  //发布的信息内容

int mode=1;  //各个阶段标志
//float realX, realY;
float k=0.1; //隧道的P
float rad;    //弧度
float degree; //角度
float range_90;  //车右侧距离
float range_91;
float range_270; //车左侧距离
const float mid_dst=0.45;  //路道宽度的一半
float theta;     //车头偏移角

float abss(float a)//这个你可以改
{
    if(a>0||a==0)
    {
       return a;
    }
    if(a<0)
    {
       return -a;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int i;
    int minX;
    int num=0;
    int count = scan->scan_time / scan->time_increment;

    for(i = 0; i < count; i++)
    {
        rad = scan->angle_min + scan->angle_increment * i;
	degree = RAD2DEG(rad);

	if(abss(degree-90) < 1)
	{
	    range_90 = scan->ranges[i];
	    range_91 = scan->ranges[i+1];
	}
	if(abss(degree-270) < 1)
	{
	    range_270 = scan->ranges[i];
	}
    }

//////////////////////////////////////////////////////////阶段判断

    if((mode==1)&&(range_90<1)&&(range_270<1))
    {
	mode=2;  //隧道
    }
    //if((mode==2)&&)

//////////////////////////////////////////////////////////隧道控制
    if(mode==2)
    {
	if(range_270 < range_90)
    	{
	    theta = acos(scan->range_min / range_270);
    	}
    	else
    	{
            theta = acos(scan->range_min / range_90);
    	}
    	if(range_90 < range_91)
        {
	    theta = -1 * theta;
    	}
    	vel_msg.linear.x=0.1;
        vel_msg.angular.z = k * theta;
    }
    /*
    minX=(scan->ranges[0]) * sin(angle_min);
    for(i = 0; i < count; i++)
    {
        rad = scan->angle_min + scan->angle_increment * i;
        realX = (scan->ranges[i]) * sin(rad);
        realY = (scan->ranges[i]) * cos(rad);
        if(abss(realX)<abss(minX))
        {
	        minX=realX;
        }
    }

    if(minX>0||minX=0)
    {
       vel_msg.angular.z=k*(mid_dst-minX);
    }
    if(minX<0)
    {
       vel_msg.angular.z=-k*(mid_dst+minX);
    }
    //k前面的正负不确定
    */

    /*for(i=count/2-angle_num;i<count/2+angle_num;i++)
    {
        rad = scan->angle_min + scan->angle_increment * i;

        if(scan->ranges[i]<mid_dst/sin(angle_carLidar))
        {
            num+=1;
        }
    }
    if(num>10&&num<30)//此时是有障碍物的时候
    {
        vel_msg.linear.x=0;
        vel_msg.linear.y=0；
        vel_msg.angular.z=0;
    }
    else if(num<10)//正常直行的时候
    {
        vel_msg.linear.x=0.1;
        vel_msg.linear.y=0；
        vel_msg.linear.z=0；

    }
    else if(num>30) //此时是靠近坡道的时候,这个时候应该怎么操作？
    {
        vel_msg.linear.y=1;//先这么来吧，这是一个无法使用的参数，但是可以作为标记，如果是1就说明到了斜坡前面，接收这个信息，调整透视变换。
        //所以listener部分你得写了
    }
*/
    
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    //vel_msg.linear.x=0.1;

    ros::Publisher vel_pub=n.advertise<geometry_msgs::Twist>("cmd",1);

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
         vel_pub.publish(vel_msg);
         ros::spinOnce();
         loop_rate.sleep();
    }
    ROS_INFO("mode, %f, %f", mode);

    return 0;
}
