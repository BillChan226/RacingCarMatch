#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
//#include <std_msgs/String.h>
#include <termios.h>///接受键盘输入
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

using namespace std;


serial::Serial ser;

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

class SimpleController
{
public:
  SimpleController();
  ~SimpleController();
  void stopRobot();
public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd);
  ros::NodeHandle nh_;

  int linear_, angular_;

  int rocker_updown = 4;  //右边的摇杆
  int rocker_leftright = 0;//左边的摇杆

  uint8_t up_speed = 0x40;
  uint8_t down_speed = 0xe0;
  uint8_t speed_scale[4] = {0x10, 0x20, 0x30, 0x40};
  int speed_ind = 3;

  //double l_scale_;
  //double a_scale_;
  ros::Publisher odo_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber cmd_sub_;

  uint8_t data_buffer[8];
  int middle_angle = 2000;///yqh: 底层将舵机的方向锁死在了800-1800之间，舵机正中间的值是1391;
  int scale_angle = 652;///yqh:4096/(2pi)

private:
/*
  function： @char2hex
  params:     @char
  return:  -1 失败 or 16进制数
  单个字节转16进制数
*/
  int char2hex(char c)
  {
      if(c >= 'a' && c <= 'f')
          return c-'a'+10;
      else if(c >= 'A' && c<='F')
          return c-'A'+10;
      else if(c >= '0' && c <= '9')
          return c-'0';
      else
          return -1;      
  }


/**
 function：hexString2byte
 params: @string 16进制字符串 @uint8_t* 16进制数组 @size 16进制字符大小
 return: 16进制字符的大小
 16进制字符串按位转成16进制字符数组，转换结果直接赋给 @uint8_t 
 
**/

  int hexString2byte(const string hex, uint8_t *bytes, int size)
  {
      int len = hex.length();
      int nbytes = (len+1)/3;
      if(nbytes > size)
          return -1;
      int n;
      for(n=0; n!=nbytes; ++n)
      {
          int lndx = n*3;
          int rndx = lndx+1;
          int lbits = char2hex(hex[lndx]);
          int rbits = char2hex(hex[rndx]);
          if(lbits == -1 || rbits == -1)
              return -1;
          bytes[n] = (lbits << 4) | rbits;
      }
      return nbytes;
  }
/**
 function：int2hexString
 params：@int
 return：@string 16进制字符串

*/
  string int2hexString(int i)
  {
      stringstream ioss;
      string s_temp;
      ioss << resetiosflags(ios::uppercase) << hex << i;
      ioss >> s_temp;
      return s_temp;
  }

/**
 fuction：calculate_check_bit
 params： @uint8_t*  串口数据包
 return： @uint8_t   校验位
 默认数据包的大小为8位
*/
  uint8_t calculate_check_bit(uint8_t* buffer)
  {//buffer[8]
      int sum = 0;
      for(int i=0; i<7; i++){
          sum += buffer[i];
      }
      string temp = int2hexString(sum);
      if(temp.length() > 2){
          temp = temp.substr(temp.size() - 2);//last two nuber
      }
      uint8_t result;
      int byte = hexString2byte(temp, &result, 1);
      return result;
  }


};


void SimpleController::stopRobot()
{
    data_buffer[0] = 0xaa;
    //set angle to middle_angle
    string t_string = int2hexString(middle_angle);
    uint8_t p1;
    uint8_t p2;
    string t_string_1 = "0"+t_string.substr(0,1);
    string t_string_2 = t_string.substr(t_string.size() - 2);
    hexString2byte(t_string_1, &p1, 1);
    hexString2byte(t_string_2, &p2, 1);
    data_buffer[1] = p1;
    data_buffer[2] = p2;
    data_buffer[3] = 0x00;//set speed to zero
    data_buffer[4] = 0x00;//set speed to zero
    data_buffer[5] = 0x00;
    data_buffer[6] = 0x00;
    data_buffer[7] = calculate_check_bit(data_buffer);
    ser.write(data_buffer,sizeof(data_buffer));
}

SimpleController::~SimpleController()
{
    stopRobot();
    usleep(10000);
    ser.close();
}

SimpleController::SimpleController():
  linear_(1),
  angular_(2)
{
    string ttyusb_port;

    ros::NodeHandle ph_nh_("~");
    ph_nh_.param<string>("ttyusb_port", ttyusb_port, string("/dev/ttyUSB1"));
    //ph_nh_.param<int>("axis_linear", linear_, linear_);
    //ph_nh_.param<int>("axis_angular", angular_, angular_);

    odo_pub_ = nh_.advertise<geometry_msgs::Twist>("odo_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SimpleController::joyCallback, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SimpleController::cmdCallback, this);
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
        stopRobot();
    }
    else
        ROS_INFO_STREAM("Serial Port Error!!!");
}


void SimpleController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int temp;
    temp = middle_angle - scale_angle*joy->axes[rocker_leftright];///yqh: 摇杆最左是1，最右是-1
    string t_string = int2hexString(temp);
    uint8_t p1;
    uint8_t p2;
    string t_string_1 = "0"+t_string.substr(0,1);
    string t_string_2 = t_string.substr(t_string.size() - 2);
    hexString2byte(t_string_1, &p1, 1);
    hexString2byte(t_string_2, &p2, 1);
    data_buffer[1] = p1;
    data_buffer[2] = p2;

    int max = 3000;//电机的扭矩对应的数值
    int speed = 0;
    int speed_l, speed_h;
    speed = 0.4 * max * joy->axes[rocker_updown];
    speed_l = speed & 0x000000ff;
    speed_h = (speed & 0x0000ff00) >> 8;
    data_buffer[3] = speed_h;
    data_buffer[4] = speed_l;

   data_buffer[7] = calculate_check_bit(data_buffer);
   ser.write(data_buffer,sizeof(data_buffer));
}

void SimpleController::cmdCallback(const geometry_msgs::Twist::ConstPtr & cmd){
    if(cmd->linear.x == 0){
        stopRobot();
        return;
    }
    
    int temp;
    temp = middle_angle - scale_angle*cmd->angular.z; //- for right turn //+ for left turn
    if(temp > 2630) temp = 2630;
    if(temp < 1410) temp = 1410;
    ROS_INFO_STREAM("temp"<<temp);
    string t_string = int2hexString(temp);
    uint8_t p1;
    uint8_t p2;
    string t_string_1 = "0"+t_string.substr(0,1);
    string t_string_2 = t_string.substr(t_string.size() - 2);
    hexString2byte(t_string_1, &p1, 1);
    hexString2byte(t_string_2, &p2, 1);
    data_buffer[1] = p1;
    data_buffer[2] = p2;
    int ratio = 3000;
    int speed = 0;
    int speed_l, speed_h;
    speed = ratio * cmd->linear.x;
    speed_l = speed & 0x000000ff;
    speed_h = (speed & 0x0000ff00) >> 8;
    data_buffer[3] = speed_h;
    data_buffer[4] = speed_l;
    data_buffer[5] = data_buffer[6] = 0;
    data_buffer[7] = calculate_check_bit(data_buffer);
    ser.write(data_buffer,sizeof(data_buffer));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_odom");
  SimpleController simple_controller;
  odom myodom;
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Time begin_time,last_time;
  last_time = ros::Time::now();
  while(ros::ok()){
      if(ser.available())
      {
            string result;//储存读入的底层电机与舵机字符串信息
            result = ser.readline(ser.available());
            ROS_INFO_STREAM("Read: " << result);
            string sub[2];
            int k=0;
            bool sub_flag = false;
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
            myodom.updateOdom(step,angular,dt);
            last_time = begin_time;
      }
      ros::spinOnce();
      loop_rate.sleep();
  }
}


















