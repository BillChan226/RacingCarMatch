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


#define RAD2DEG(x) ((x)*180./M_PI)


ros::Publisher pub;
geometry_msgs::Twist laser_cmd;

// laser_cmd.linear.z > 0  = enable laser control
// laser_cmd.linear.z < 0  = disable laser control
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  
    /*
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //if (degree < -175);
            //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    */

    //left side
    
    float x,y,rad;
    float y_nearest_left = -10;
    for(int i = 0; i < count/4; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        float rad = scan->angle_min + scan->angle_increment * i;
        y = scan->ranges[i] * sin(rad);
        //ROS_INFO("%f, %f", degree, scan->ranges[i]);
        //ROS_INFO("%d:", count);
        x = scan->ranges[i] * cos(rad);
        if (x > -1.00 && y > y_nearest_left)
            y_nearest_left = y;   
    }
    //right side

    float y_nearest_right = 10;
    for(int i = count*3/4; i < count; i++) {
        rad = scan->angle_min + scan->angle_increment * i;
        y = scan->ranges[i] * sin(rad);
        x = scan->ranges[i] * cos(rad);
        if (x > -1.00 && y < y_nearest_right)
            y_nearest_right = y ;   
    }
    
    if (y_nearest_left < 0 && y_nearest_right > 0 && (y_nearest_right - y_nearest_left < 1.00))
    {
        laser_cmd.linear.z = 1.0;
        ROS_INFO("laser control");
        float k = 1.0;
        laser_cmd.angular.z = k * (y_nearest_right + y_nearest_left);
    }
    else
    {
        laser_cmd.linear.z = -1.0;
        ROS_INFO("camera control");
    }



    

    ////////pedestrian detection

    bool obstacle_width[20] = {false};
    int index = 0;
    float nearest_x = -0.9;
    int current;
    for(int i = count*9/10; i <= count*11/10; ++i)
    {
        current = i%count;
        rad = scan->angle_min + scan->angle_increment * current;
        x = scan->ranges[current] * cos(rad);
        y = scan->ranges[current] * sin(rad);
        if (obstacle_width[index] == false)
        {
            if (x > -0.9)//-0.9)
            {
                if (y < 0.25 - index * 0.025 && y > 0.25 - (index + 1) * 0.025)
                    obstacle_width[index] = true;
                if (x > nearest_x)
                    nearest_x = x;
            }   
        }
        if (obstacle_width[index] == true || (y < 0.25 - (index + 1) * 0.025 && x > -0.9))
            ++index;
        if (index == 20)
            break;
    }
 
    int width_count = 0;
    for(int i = 0; i < 20; ++i)
    {
         width_count += obstacle_width[i];
    }
    
    if (width_count > 2 && width_count < 15)
    {
         laser_cmd.linear.x = 0.0;
         ROS_INFO("pedestrian______________________");
    }
    else if (width_count <= 2)
         laser_cmd.linear.x = 0.3;
    else
         ROS_INFO("ramp______________________");
    
    
    if (width_count >= 2)
       laser_cmd.linear.x = 0.0;
    else
       laser_cmd.linear.x = 0.3;
       
    
    

     

    pub.publish(laser_cmd);

    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    pub = n.advertise<geometry_msgs::Twist>("laser_cmd", 1);

    

    laser_cmd.linear.x = 0.3;

    ros::spin();

    return 0;
}
