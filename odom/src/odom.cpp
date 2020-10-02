#include <ros/ros.h>
#include <serial/serial.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <sys/poll.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
serial::Serial ser;
using namespace std;
class odom{
public:
	double odom_x;
	double odom_y;
	double odom_quat; 
	int odom_step; // m 电机总步数
	double odom_speed;//   m/s
    double odom_psi; //偏航角
	double odom_last_speed = 0.0;
	double step2round = 4000.0; //轮胎转一圈对应的总步数
	double odom_perimeter = 0.5338;//m 轮胎周长
	double odom_wheel = 0.30; //m 后轮间距
    double odom_front_overhang = 0.40; //m 前悬长度
    double odom_rear_overhang = 0.20; //m 后悬长度
    double middle_angle = 2047.0;
	odom();
	~odom();
	void updateOdom(int step,int angular,double dt);
	ros::Publisher odom_pub;
	ros::NodeHandle n;
};
odom::odom(){
	odom_x = 0.0;
	odom_y = 0.0;
	odom_quat = 0.0;
	odom_speed =0.0;
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
/*
    string ttyusb_port;
    ros::NodeHandle n("~");
    n.param<string>("ttyusb_port", ttyusb_port, string("/dev/ttyUSB1"));
    try
    {
        ser.setPort(ttyusb_port.c_str());
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM(string("Unable to open port ")+ttyusb_port);
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM(string("Serial Port initialized")+ttyusb_port);
    }
    else
        ROS_INFO_STREAM("Serial Port Error!!!");
*/
}
odom::~odom()
{

}
void odom::updateOdom(int step,int angular,double dt){
    /**
     * @brief odometry process
     * β=tan−1(lr/(lf+lr)*tan(δf))  β:滑移角  lr: 后悬长度  lf:前悬长度 δf: 前轮偏角，我这里就用的舵机转角
     * ψt+1=ψt+vt/(lr+lf)*sin(β)×dt    ψt: 航向角，车辆实际角度
     * y(t+1)=yt+vt*sin(ψt+β)×dt
     * x(t+1)=xt+vtcos(ψt+β)×dt 
     **/
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    //计算步数变化量
	int delta_step = step - odom_step;
	odom_speed = ((delta_step/(step2round))*odom_perimeter)/dt;
	//计算滑移角beta
    double beta = atan(((odom_rear_overhang+odom_front_overhang)/odom_front_overhang)*tan(angular));
    //计算偏航角
    odom_psi = odom_psi + (odom_speed)*cos(beta)/(odom_rear_overhang+odom_front_overhang)*tan(angular)*dt;

    //计算下x,y位移
    odom_x = odom_x + abs(odom_speed)*cos(odom_psi + beta)*dt;
    odom_y = odom_y + abs(odom_speed)*sin(odom_psi + beta)*dt;
    ROS_INFO_STREAM("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh");

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_psi);

    //first, we'll publish the transform over tf
    current_time = ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //publish the message
    odom_step = step;
    odom_last_speed = odom_speed;
    odom_pub.publish(odom);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  odom my_odom;
  ROS_INFO_STREAM("sssssssssssssssssssssssssssssss");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Rate r(100);
  ros::Time begin_time,last_time;
  last_time = ros::Time::now();
  while(n.ok()){
    if(ser.available()){
            ROS_INFO_STREAM("startttttttttttttttttttttttttttttt");
            string result;
            result = ser.readline(ser.available());
            //sub[0] 里的是电机的步数
            //sub[1] 里的是舵机的角度
            string sub[2];
            int k=0;
            bool sub_flag = false;
            //从串口读取电机步数和舵机角度
            for(int i=0; i<result.length(); i++){
                if(result[i] == ':'){
                    sub_flag = true;

                }
                if(result[i] == '='){
                    sub_flag = false;
                    k++;

                }
                if(sub_flag && result[i] != '=' && result[i] != ':'){
                    sub[k] += result[i];
                }
    }
    int step = atoi(sub[0].c_str());
    int angular = atoi(sub[1].c_str());
    ros::Time begin_time,last_time;
    begin_time = ros::Time::now();
    double dt = (begin_time - last_time).toSec(); 
    my_odom.updateOdom(step,angular,dt);
    last_time = begin_time;
    ros::spinOnce();
    r.sleep();
}
}
}
