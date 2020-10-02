#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>                                                                          //这个serial类是自己写的吗？有无相关说明？
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

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

using namespace std;


serial::Serial ser;
float old_speed = 0;


class SimpleController
{
public:
  SimpleController();
  ~SimpleController();
  void stopRobot();


  //智能车的尺寸数据，暂时不用管
  //Odometry related
  double odo_perimeter = 53.38;//cm
  int odo_wheelbase = 60;//cm
  int odo_front_overhang = 40; //cm
  int odo_rear_overhang = 20; //cm
  double odo_speed = 0.0;//     cm/s
  double odo_steer_angular = 0.0;                                                                                          //单位是度吗？
  int odo_round = 0;
  double odo_motor_angular = 0.0;                                                                                          //单位是度吗？

  double odo_x = 0.0;
  double odo_y = 0.0;
  double odo_psi = 0.0;
  double odo_last_speed = 0.0;
  int odo_times = 0;
  void updateOdometry(int steer_angular, int motor_angular, int round);                                          //功能？


public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd);
  ros::NodeHandle nh_;

  int linear_, angular_;

  int rocker_updown = 4;  //右边的摇杆（遥杆是指手柄控制的遥杆，现在暂时用不到）                                                                                
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
  int middle_angle = 2237;///yqh: 底层将舵机的方向锁死在了800-1800之间，舵机正中间的值是1391;                                                      //所以为啥middle_angle是2237？而不是800-1800之间的某个数
  int scale_angle = 652;///yqh:4096/(2pi)                                                              //scale_angle是什么？ 652的含义？

private:
/*
  function： @char2hex
  params:     @char
  return:  -1 失败 or 16进制数
  单个字节转16进制数
  若遇到@char类型的数字，则直接将其转化为十六进制的值返回，即返回值0x0—0x9表示的是数字，eg:输入“2”，输出0x2；
  若遇到的是@char类型的字母，不区分大小写，返回值是字母序号+10，即0x10—0x20表示的是字母，eg:输入“A”/“a”,返回0x10
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
 function：int2hexString   输入@int i 将i转化为@string 类型的十六进制字符串输出
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
@unit8_t类型8位data_buffer的数组第[7]位为校验位，为前7位的和 
 fuction：calculate_check_bit    将data_buffer前七个两位十六进制数加起来，截取最后两位十六位数，以@unit8_t类型返回                                                                                                //是奇校验还是偶校验
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
      string temp = int2hexString(sum);//将@int类型的sum转化为十六进制并以@string类型存储
	  //考虑到data_buffer一个元素只能存储8位二进制即2位16进制数，而temp的值可能多余两位（ > EE)，因此需截取temp中的末两于temp中                                                                                       
	  if(temp.length() > 2){
          temp = temp.substr(temp.size() - 2);//提取temp的最后两位，存入temp中
      }
      uint8_t result;
      int byte = hexString2byte(temp, &result, 1);//将@string的两位校验位转换为16进制数存入result中
      return result;
  }


};

/*
data_buffer数组元素
关于@unit8_t类型8位data_buffer的数组元素：
使用USART_RX_BUF通信协议：第[0]位是字头0xaa;第1-6位是数据；第[7]位为校验位
第[1]位表示舵机高八位；第[2]位表示舵机低八位；
第[3]位为电机高八位；第[4]位为电机低8位；
第[5]、[6]位为拓展预留位；
第[7]位为校验位，为前7位的和的末两位（16进制）                                                                              
*/


//func：通过修改data_buffer里面的数据，使舵机保持中位，电机速度为0，并将data_buffer的数据发送出去
void SimpleController::stopRobot()                                                                      
{
    data_buffer[0] = 0xaa;//第一位为字头，规定0xaa
    //set angle to middle_angle
	//t_string为舵机保持中位的@string类型的数据，以下7行是将@int类型的middle_angle=2237转化为二进制数，高八位存入p1,低八位存入p2
    string t_string = int2hexString(middle_angle);//将@int类型的middle_angle = 2237转换为@string类型的16进制数存入t_string中，t_string = 8BD
    uint8_t p1;
    uint8_t p2;
	string t_string_1 = "0" + t_string.substr(0, 1);//t_string_1为t_string的前两位                                            //t_string一共几位确定吗？为何要加‘0’？                                                                    //string t_string_1 怎么算？
    string t_string_2 = t_string.substr(t_string.size() - 2);//t_string_2为t_string的末两位,t_string_2 = BD
    hexString2byte(t_string_1, &p1, 1);//将@string类型的16进制数t_string_1转换为@unit8_t 的二进制数存入p1中
    hexString2byte(t_string_2, &p2, 1);//将@string类型的16进制数t_string_2转换为@unit8_t 的二进制数存入p2中

    data_buffer[1] = p1;//舵机高八位  将@string类型的p1转换为@unit8_int类型的8位二进制数存入data_buffer[1]中
    data_buffer[2] = p2;//舵机低八位  将@string类型的p2转换为@unit8_int类型的8位二进制数存入data_buffer[2]中
    data_buffer[3] = 0x00;//电机高八位   set speed to zero
    data_buffer[4] = 0x00;//电机低8位  set speed to zero
    data_buffer[5] = 0x00;//拓展预留位，没用
    data_buffer[6] = 0x00;//拓展预留位，没用
    data_buffer[7] = calculate_check_bit(data_buffer);//校验位
    ser.write(data_buffer,sizeof(data_buffer));//以ascII码的形式发送data_buffer的8位@unit8_t的数据                                                      //谁来接收？
}

/*
析构函数
将舵机摆正，电机速度设为0 

关闭通信串口
*/
SimpleController::~SimpleController()//析构函数
{
    stopRobot();                                                                                                                    
    usleep(10000);//休眠10ms
	/*
	    两个节点的通过topic建立起来联系是需要时间的．
	如果没有休眠一下，可以说程序一跑起来瞬间消息就发布出去了．
	但是我们接下来要写的另一个节点和这个节点还没连接起来，
	说白了就是ROS没反应过来．那这个消息就漏掉了
	*/
    ser.close();//关闭串口
}

//构造函数
SimpleController::SimpleController():
  linear_(1),                                                                                            //??
  angular_(2)                                                                                            //??
{
    string ttyusb_port;

    ros::NodeHandle ph_nh_("~");//创建NodeHandle类型的变量ph_nh                                          //括号了里是什么意思？
	/*
	默认构造函数  
	NodeHandle(const std::string& ns = std::string(), const M_string& remappings = M_string());//
	*/

    ph_nh_.param<string>("ttyusb_port", ttyusb_port, string("/dev/ttyUSB0"));                          //为何多了个<string>?
	/*
	ros::nodehandle::param(const std::string & param_name,T & param_val,const T & default_val )
    此方法尝试从参数服务器检索指示的参数值，并将结果存储在param_val中。
	param_name:参数服务器中的参数名  此处参数服务器为"ttyusb_port"
    param_val：将该参数名的值读取到param_val。  此处将参数服务器"ttyusb_port"传来的值存入@string类型的变量 ttusb_port
    default_val：指定默认的值。     此处，若无法从服务器检测到"ttusb_port"则使用默认值"/dev/ttyUSB0"
	*/

    //ph_nh_.param<int>("axis_linear", linear_, linear_);
    //ph_nh_.param<int>("axis_angular", angular_, angular_);

    odo_pub_ = nh_.advertise<geometry_msgs::Twist>("odo_vel", 1);
	/*
	发布话题，返回一个Publisher，负责广播topic
	尖括号“<>"中填写advertise的信息类型
    Publisher advertise (const std::string &topic, uint32_t queue_size, bool latch=false)
	topic:指定pubulisher往哪个topic发布消息        此处向"odo_vel"这个topic发布消息
	queue_size:发送的消息队列的长度    此处设定为1
	返回值为@publisher类型                                                                                  //返回的内容是什么？
	*/
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SimpleController::joyCallback, this);
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SimpleController::cmdCallback, this);
	/*
	订阅一个话题，收到话题中的消息后触发回调函数
	尖括号“<>"中填写需要subscribe的信息类型
	Subscriber subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints=TransportHints())
	topic:订阅的topic的名称，即从这个topic上接收信息  此处从"cmd_vel"这一topic上接收消息
	queue_size:接收的消息队列的长度    此处设定为1
	obj:回调函数的名称   回调函数的作用：当接收到发布者的信息后，需进行一系列的处理，处理过程就编写在回调函数中，而此处的this指针？？？                                                                             
	返回值为@subscriber类型                                                                                  
	*/
  
	/*
   异常处理语句try...catch...
   基本思想是：函数 A 在执行过程中发现异常时可以不加处理，
   而只是“拋出一个异常”给 A 的调用者，假定为函数 B。拋出
   异常而不加处理会导致函数 A 立即中止，在这种情况下，函数 B
   可以选择捕获 A 拋出的异常进行处理，也可以选择置之不理。如
   果置之不理，这个异常就会被拋给 B 的调用者，以此类推。如果
   一层层的函数都不处理异常，异常最终会被拋给最外层的 main 
   函数。main 函数应该处理异常。如果main函数也不处理异常，那么程序就会立即异常地中止。

   try...catch 语句的执行过程是：
    执行 try 块中的语句，如果执行的过程中没有异常拋出，那么执行完后就执行最后一个 catch 块
后面的语句，所有 catch 块中的语句都不会被执行；
如果 try 块执行的过程中拋出了异常，那么拋出异常后立即跳转到第一个“异常类型”和拋出的异
常类型匹配的 catch 块中执行（称作异常被该 catch 块“捕获”），执行完后再跳转到最后一个 catch 块后面继续执行。
   try
{
	//可能抛出异常的语句
}
catch (异常类型1)
{
	//异常类型1的处理程序
}
catch (异常类型2)
{
	//异常类型2的处理程序
}
// ……
catch (异常类型n)
{
	//异常类型n的处理程序
}

   */
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
        ROS_ERROR_STREAM(string("Unable to open port ")+ttyusb_port);                                              //???string("Unable to open port ")+ttyusb_port
		/*
		ROS中,日志消息分为五个不同的严重级别,也可简称为严重性或者级别。按照严重性程度递增,这些级别有
       DEBUG
       INFO
       WARN
       ERROR
       FATAL
	   
	   ROS_XXX_STREAM（message）相当于cout
		*/
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM(string("Serial Port initialized")+ttyusb_port);////ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台,相当于cout                         //???

        stopRobot();
    }
    else
        ROS_INFO_STREAM("Serial Port Error!!!");////ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台  此处控制台上显示“ Serial Port Error!!!”
}

//与游戏手柄操作有关，可以先不管
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
    speed = 0.4*max * joy->axes[rocker_updown];
    speed_l = speed & 0x000000ff;
    speed_h = (speed & 0x0000ff00) >> 8;
    data_buffer[3] = speed_h;
    data_buffer[4] = speed_l;

   data_buffer[7] = calculate_check_bit(data_buffer);
   ser.write(data_buffer,sizeof(data_buffer));
}


/*
data_buffer数组元素
关于@unit8_t类型8位data_buffer的数组元素：
使用USART_RX_BUF通信协议：第[0]位是字头0xaa;第1-6位是数据；第[7]位为校验位
第[1]位表示舵机高八位；第[2]位表示舵机低八位；
第[3]位为电机高八位；第[4]位为电机低8位；
第[5]、[6]位为拓展预留位；
第[7]位为校验位，为前7位的和的末两位（16进制）
*/

/*
input:@geometry_msgs::Twist::ConstPtr & cmd
func:接收  包的控制信息
     若控制信息中x方向的速度为0，则执行stopRobot();
	 若控制信息中x方向的速度不为0，则1.将控制信息里的舵机转角换算为data_buffer格式的舵机转角信息存入data_buffer用于存储多级信息的1、2位
	                                 2.将控制信息里的电机速度换算为data_buffer格式的电机速度信息存入data_buffer用于存储多级信息的3、4位
									 3.发送data_buffer
原理：                                                                                                        //???
*/
void SimpleController::cmdCallback(const geometry_msgs::Twist::ConstPtr & cmd){
    if(cmd->linear.x == 0)//若机器人x方向上的线速度为0                                                        //linear x,y,z   angular x,y,z分别是哪三个方向？
	{
        stopRobot();//通过修改data_buffer里面的数据，使舵机保持中位，电机速度为0，并将data_buffer的数据发送出去
        return;
    }
    
    int temp;//temp用于储存计算后的舵机转角信息
    temp = middle_angle - scale_angle*cmd->angular.z; //- for right turn //+ for left turn             //这句没看懂

    if(temp > 2793) temp = 2793;                                                                       //2793??
    if(temp < 1427) temp = 1427;   //1410                                                              //1410??
    ROS_INFO_STREAM("temp"<<temp);////ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台              //???

    //将@int类型的temp转化为二进制数，高八位存入p1,低八位存入p2   而p1对应舵机高八位，p2对应舵机低八位
	string t_string = int2hexString(temp);
    uint8_t p1;
    uint8_t p2;
    string t_string_1 = "0"+t_string.substr(0,1);
    string t_string_2 = t_string.substr(t_string.size() - 2);
    hexString2byte(t_string_1, &p1, 1);
    hexString2byte(t_string_2, &p2, 1);
	
    data_buffer[1] = p1;
    data_buffer[2] = p2;

    //int ratio = 16384/(15.63*odo_perimeter);//电机的扭矩电流 -16384~0~16384 对应 -20A~0~20A         //15.63怎么来的？ratio算出来有什么物理意义？
    int ratio = 3000;
    int speed = 0;//speed用于存储计算后的电机速度信息
    int speed_l, speed_h;
//    if (fabs(cmd->linear.x - old_speed) >= 0.04)
  //  {
    //   old_speed += 0.02*(cmd->linear.x-old_speed)/fabs(cmd->linear.x-old_speed);
      // speed = ratio * old_speed;
    //}
    //else
       speed = ratio * cmd->linear.x;//求出的speed为                                                 //speed的物理意义？
       
   // old_speed=speed/ratio;
    speed_l = speed & 0x000000ff;//speed_1相当于先将@int类型的speed转为二进制数，取最低8位，再转回@int
    speed_h = (speed & 0x0000ff00) >> 8;//speed_h相当于先将@int类型的speed转为二进制数，取第8-15位（第0位为最低位），再转回@int
    data_buffer[3] = speed_h;//按照data_buffer的通信格式，data_buffer[3]存储电机高八位的信息
    data_buffer[4] = speed_l;//按照data_buffer的通信格式，data_buffer[4]存储电机低八位的信息
    data_buffer[5] = data_buffer[6] = 0;
    data_buffer[7] = calculate_check_bit(data_buffer);//计算校验位
    ser.write(data_buffer,sizeof(data_buffer));//以ascII码的形式发送data_buffer的8位@unit8_t的数据 
}



/**
    steer_angular：舵机扭转的角度
    motor_angular：轮胎旋转的角度
    round：        轮胎旋转的圈数  正转会增加，反转会减小
    speed = (△motro_angular/180+△round)*后轮周长
    
*/

//与里程计相关的函数，现在暂时用不到
void SimpleController::updateOdometry(int steer_angular, int motor_angular, int round){

    double differ = middle_angle-steer_angular;   //middle_angle是simple_controller类参数  middle_angle =2237  
    //因为左右两边的扭矩和角度对应不同，这里简单的做了个归一化处理                                                                           //归一化的作用是什么？什么是归一化？
    odo_steer_angular = (differ>0 ? (3.14/180*(differ/500)*27.5) : (3.14/180*(differ/350)*27.5));                                            //500?350?27.5? 分别是怎么来的？
    odo_speed = ((motor_angular-odo_motor_angular)/180+(round - odo_round))*odo_perimeter;//SimpleController 类中定义的参数:odo_motor_angular=0  odo_round=0  odo_perimeter=53.38
    odo_speed = odo_speed<2 ? odo_speed : 0;  //限幅，有时候会读到一个很大的跳变                                                             //既然是限幅，为何是<2时 = 0？

    //test                                                                                                                                   //如何测试？
    geometry_msgs::Twist twist; //测试里程计  因为模型不同，这个测不出来  我用的小乌龟测
    twist.angular.z = odo_steer_angular;
    twist.linear.x = odo_speed;
    odo_pub_.publish(twist);
    //~test

    /**
     * @brief odometry process
     * β=tan−1(lr/(lf+lr)*tan(δf))  β:滑移角  lr: 后悬长度  lf:前悬长度 δf: 前轮偏角，我这里就用的舵机转角
     * ψt+1=ψt+vt/(lr+lf)*sin(β)×dt    ψt: 航向角，车辆实际角度
     * y(t+1)=yt+vt*sin(ψt+β)×dt
     * x(t+1)=xt+vtcos(ψt+β)×dt
     *
     * 
     */
    double beta = atan((double)2/3*tan(odo_steer_angular));
    odo_psi = odo_psi + (odo_speed)*cos(beta)/60.0 * tan(odo_steer_angular);
    double angular = fmod(odo_psi,3.14);
   
    //这一段是因为串口读取速度的原因，会造成前几帧都有残余数据，从而造成偏差，舍弃前10帧数据就好了
    if(odo_times > 10){
        
        double delta_speed = (odo_last_speed - odo_speed) > 0 ? (odo_last_speed - odo_speed) : (odo_speed - odo_last_speed);
        delta_speed = delta_speed < 1 ? delta_speed : 0;
        
        odo_x = odo_x + abs(odo_speed)*cos(odo_psi+beta)/100;
        odo_y = odo_y + abs(odo_speed)*sin(odo_psi + beta)/100;
       
    }else{
        odo_times++;
    }


    //~odometry process


    odo_motor_angular = motor_angular;
    odo_round = round;
    odo_last_speed = odo_speed;
}


int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Start!");////ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台
  ros::init(argc, argv, "simple_controller");//解析参数，命名节点为"simple_controller"
  /*
  ros::init()是ROS程序调用的第一个函数，用于对ROS程序的初始化
  第一第二个参数为argc argv;第三个参数为当前节点命名
  */
  SimpleController simple_controller;//创建句柄(提供了对node操作的方法)，实例化node
  ros::Rate loop_rate(100);//设置频率为100hz
  //ros::Rate  它的功能就是先设定一个频率，然后通过睡眠度过一个循环中剩下的时间，来达到该设定频率。如果能够达到该设定频率则返回true，不能则返回false。                                                                                             //？？？high_resolution_clock  time_point
  
  high_resolution_clock::time_point beginTime;
  high_resolution_clock::time_point endTime;
  int isFirst = 1;
  while(ros::ok())
  /*
  ros::ok()返回false，代表可能发生了以下事件		
  //1.SIGINT被触发(Ctrl-C)调用了ros::shutdown()		
  //2.被另一同名节点踢出 ROS 网络		
  //3.ros::shutdown()被程序的另一部分调用		
  //4.节点中的所有ros::NodeHandles 都已经被销毁
  也就是说只要ros还没有关或者异常，就会一直进行循环
  */
  {
      ROS_INFO_STREAM("Ros ok!");////ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台，此处控制台显示“Ros ok!”
      if(ser.available())
	  {
            ROS_INFO_STREAM("Reading to read!");//ROS_INFO_STREAM将会生成一条消息，并且会发送到控制台，此处控制台上显示“Reading to read!”
            string result;
            result = ser.readline(ser.available());                                                                            //???result的内容
            ROS_INFO_STREAM("Read: " << result);//若此时result内存储的是“ABC”,则控制台上会显示“Read:ABC”
           
		                                                                                                           //?这段没看懂
		    //string str = "P:3143=cnt:12580=cnt1:88779=";
            //sub[0] 里的是舵机的角度
            //sub[1] 里的是电机的角度
            //sub[2] 里的是电机旋转的圈数
            string sub[3];
            int k=0;
            bool sub_flag = false;
            beginTime = high_resolution_clock::now();//high_resolution_clock::now()记录了当前的时刻

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

            milliseconds timeInterval = std::chrono::duration_cast<milliseconds>(beginTime - endTime);//timeInterval单位是毫秒， 记录了beginTime到endTime的时间间隔        //?endTime怎么没赋值
            //if(endTime != NULL)
            int t = int(timeInterval.count());//count()的功能可能是将@milliseconds 的timeInterval变量中表示时间段的@int类型数字提取出来，这个数字的单位是ms                                                 //?count函数功能？
            ROS_INFO_STREAM("MS: " << t);
            endTime = beginTime;

            //odometry process
            int odometry_steer = atoi(sub[0].c_str());
            int odometry_motor = atoi(sub[1].c_str());
            int odometry_round = atoi(sub[2].c_str());
			/*
			原型：int atoi(const char *nptr);
			编辑参数nptr字符串，如果第一个非空格字符存在，是数字或者正负号则开始做类型转换，
			之后检测到非数字(包括结束符 \0) 字符时停止转换，返回整型数。否则，返回零，
			*/

            //ROS_INFO_STREAM("first: " << isFirst);
            if(!isFirst){
                simple_controller.updateOdometry(odometry_steer,odometry_motor,odometry_round);
            }
            isFirst = 0;
      }
      ros::spinOnce();//查看并调用一次当前可触发（即存储接收信息的队列非空）的subscribe函数对应的回调函数，非阻塞。即若subscribe函数对应的存储信息的队列非空，就会调用一次回调函数，并且清空队列。当所有可以调用的回调函数调用完了，则进行下一步操作
	  //还有一个相似的函数ros::spin():反复查看并调用当前可触发（即存储接收信息的队列非空）的subscribe函数对应的回调函数，阻塞。即查看各subscriber,若subscribe函数对应的存储信息的队列非空，就会调用一次回调函数，并且清空队列。其会反复查看各subscriber,不会执行下一步程序
	  //若缺少spin()或spinOnce()则回调函数无法触发，即回调函数须与spin()或spinOnce()组合使用
	  /*
	  ROS消息回调处理函数
	      在ROS的主循环中，程序需要不断调用ros::spin() 或 ros::spinOnce()，两者区别在于前者调用后不会再返回，
	  也就是你的主程序到这儿就不往下执行了，而后者在调用后还可以继续执行之后的程序。
	      如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你
	  规定的格式，接受订阅的消息，但是所接到的消息并不是立刻就被处理，而是必须要等到ros::spin()或
	  ros::spinOnce()执行的时候才被调用
	  */

      loop_rate.sleep();//在一个周期内处理完上述进程之后的剩余时间内休眠，直到下一个周期开始
  }
}


















