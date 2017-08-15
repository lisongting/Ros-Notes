# Ros C++代码理解


最简单的发布器：Publisher------------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  //ros的初始化函数，第三个参数是节点名称
  ros::init(argc, argv, "talker");

  //NodeHandle： n是用来与Ros系统通讯的节点实例
  ros::NodeHandle n;

  //通过调用n.advertise()函数：
  //创建一个Publisher对象，指定Message类型为std::String，topic为"chatter"，
  //1000表示Message缓存队列的长度，超过1000条消息之后，旧的消息就会丢弃。
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  //发布频率：10次/秒
  ros::Rate loop_rate(10);

  int count = 0;

  //ros::ok()一般会返回true，只在以下情况返回false：
//  1.SIGINT收到(Ctrl-C)信号
// 2.另一个同名节点启动，会先中止之前的同名节点
// 3.ros::shutdown()被调用
// 4.所有的ros::NodeHandles被销毁
  while (ros::ok())
  {
    //创建Message对象
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    //通过调用Publisher对象的publish(),将指定的Message发布出去
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}


最简单的订阅器：Subscriber------------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"

//订阅节点的回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
//初始化一个节点，名为"listener"
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

 //订阅的topic为"chatter"，订阅的回调函数为chtterCallback
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  //启动循环，将这个节点置入运行状态
  ros::spin();
  return 0;
}



最简单的服务端（用来提供Service）------------------------------------------------------------------------
#include "ros/ros.h"
//AddTwoInts.h是编译系统根据写好的srv文件自动生成的
#include "beginner_tutorials/AddTwoInts.h"

//req代表请求对象    
//res代表响应对象
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
 //初始化一个节点，叫做"add_two_ints_server"
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

//使用n.advertiseService()函数，表示注册了一个叫做"add_two_ints"的Service，处理函数为add
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}



最简单的客户端 (用来使用Service)------------------------------------------------------------------------
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  //创建节点
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  //创建ServiceClient，指定srv类型为AddTwoInts，调用名称为"add_two_ints"的Service
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");

  //构造请求对象
  beginner_tutorials::AddTwoInts srv;
  //atoll把字符串转换为长整型
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  //调用call()，这个一个阻塞式方法，调用完成则返回true，失败则返回false
  if (client.call(srv))
  {
  //srv.response 就是响应对象，输出响应对象
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  return 0;
}


与参数相关的操作------------------------------------------------------------------------
使用NodeHandle对象来进行与参数相关的操作
ros::NodeHandle n;

//获取参数
//key是参数名，output_value用来保存参数的值 , 
//getParam()的返回值是bool,可以通过返回值来判断参数是否获取成功
n.getParam(const std::string & key, parameter_type & output_value );
//如：
std::string s;
n.getParam("my_param",s);

//可以使用param()函数来获取参数，可以附带默认值
int i ;
n.param("num",i,6);
n.param<std::string>("my_param",s,"default_value");


//设置参数
n.setParam("my_param","hello");

//删除参数
n.deleteParam("my_param");

//检查参数
if(!n.hasParam("my_param")){
    ROS_INFO("No Param named 'my_param' ");
}

//搜索参数
 std::string param_name;
  if (n.searchParam("b", param_name))
  {
    // Found parameter, can now query it using param_name
    int i = 0;
    n.getParam(param_name, i);
  }
  else
  {
    ROS_INFO("No param 'b' found in an upward search");
  }

NodeHandle  节点类------------------------------------------------------------------------
ros::NodeHandle 有两个作用：
    * 首先，在roscpp程序内部，它提供了RAII（资源请求即初始化）方式启动和关闭
    * 其次，它提供了一个额外的层命令空间解决方案，可以使组件更容易写

---------访问NodeHandle的私有名称---------
使用场景：创建了多个节点后，用来获取某个节点的参数
ros::init(argc,argv,"my_node_name");
ros::NodeHandle nh1("~");
ros::NodeHandle nh2("~foo");

nh1的命名空间是/my_node_name
nh2的命名空间是/my_node_name/foo

则以下两种访问参数的形式等价：
//方式1
ros::NodeHandle nh;
nh.getParam("~name", . . . .);

//方式2
ros::NodeHandle nh("~");
nh.getParam("name", . . . .);




使用类的方法作为服务的回调函数------------------------------------------------------------------------
class Listener{
    public :
        void callback(const std_msgs::String::ConstPtr &msg);
};

//如果使用普通方法作为回调
ros::Subscriber sub = n.subscribe("chatter",1000,func);

//如果使用类方法作为回调
Listener listener;
ros::Subscriber sub = n.subscribe("chatter",1000,&Listener::callback, &listener);



使用定时器 (Timers)类------------------------------------------------------------------------
#include "ros/ros.h"

void callback1(const ros::TimerEvent& event)
{
  ROS_INFO("Callback 1 triggered");
}

void callback2(const ros::TimerEvent&  event)
{
  ROS_INFO("Callback 2 triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  //创建两个定时器，timer1是0.1秒执行一次，执行后的回调是callback1
  //timer2是1秒执行一次，执行后的回调是callback2
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);

  //Timer还有一种创建方法：
  //其中，第三个参数代表oneShot，false表示一直执行，true表示只执行一次
  ros::Timer timer3 = n.createTimer(ros::Duration(1),callback1,false)
  ros::spin();

  return 0;
}

//TimerEvent结构体：作为参数传入，它提供时间的相关信息，对于调试和配置非常有用
struct TimerEvent
{
  Time last_expected;                     //上次回调期望发生的时间
  Time last_real;                              //上次回调实际发生的时间

  Time current_expected;               //本次回调期待发生的时间
  Time current_real;                        //本次回调实际发生的时间

  struct
  {
    WallDuration last_duration;       //上次回调的时间间隔（结束时间-开始时间），是wall-clock时间。
  } profile;
};



高级的发布器------------------------------------------------------------------------

-----------动态配置文件---------
//cfg文件夹下的node_example.cfg文件
//这是可动态配置的，可以在运行过程中更改message,a和b的值
//？？？
#! /usr/bin/env python

PACKAGE='node_example'
import roslib
roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
#       Name       Type      Level Description     Default Min   Max
gen.add("message", str_t,    0,    "The message.", "hello")
gen.add("a",       int_t,    0,    "First number.", 1,     -100, 100)
gen.add("b",       int_t,    0,    "First number.", 2,     -100, 100)

exit(gen.generate(PACKAGE, "node_example", "nodeExample"))


-----------发布器----------
#include <node_example/talker.h>

namespace node_example
{
ExampleTalker::ExampleTalker(ros::NodeHandle nh) : message_("hello"), a_(1), b_(2), enable_(true)
{
 //设置一个动态的重配置服务器
//需要在参数服务器之前设置这个重配置服务器，否则一些参数就会被参数服务器复写
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  //这一行是什么意思？？
  cb = boost::bind(&ExampleTalker::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  //声明的变量可以被launch文件或命令行更改
  int rate;

  //初始化一个NodeHandle ,使用私有的名称，这样可以确保多个节点可以在不同的参数下同时运行
  ros::NodeHandle pnh("~");
  //获取参数
  pnh.param("a", a_, a_);
  pnh.param("b", b_, b_);
  pnh.param("message", message_, message_);
  pnh.param("rate", rate, 1);
  pnh.param("enable", enable_, enable_);

  pub_ = nh.advertise<node_example::NodeExampleData>("example", 10);
//创建定时器，实际上就是每触发一次定时器，则发布一次消息
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);
}


void ExampleTalker::timerCallback(const ros::TimerEvent &event)
{
  if (!enable_)
  {
    return;
  }
  node_example::NodeExampleData msg;
  msg.message = message_;
  msg.a = a_;
  msg.b = b_;

  pub_.publish(msg);
}

//重配置服务器的回调函数，当进行参数重配置时，就会触发该函数，然后设置相应的值
void ExampleTalker::configCallback(node_example::nodeExampleConfig &config, uint32_t level)
{
  message_ = config.message;
  a_ = config.a;
  b_ = config.b;
  enable_ = config.enable;
}
}

-----------发布器的头文件：talker.h------
#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H
#include <ros/ros.h>
#include <ros/time.h>
#include <node_example/NodeExampleData.h>
#include <dynamic_reconfigure/server.h>
#include <node_example/nodeExampleConfig.h>

namespace node_example
{
class ExampleTalker
{
 public:  
  explicit ExampleTalker(ros::NodeHandle nh);
  void configCallback(node_example::nodeExampleConfig &config, uint32_t level);
  void timerCallback(const ros::TimerEvent &event);

 private:
  ros::Timer timer_;
  ros::Publisher pub_;
  //声明动态重配置服务器
  dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;
  std::string message_;

  int a_;
  int b_;
  bool enable_;
};
}
#endif  


高级的订阅器------------------------------------------------------------------------
#include "node_example/listener.h"

namespace node_example
{
    ExampleListener::ExampleListener(ros::NodeHandle nh)
    {
        //用类的方法作为回调函数
      sub_ = nh.subscribe("example", 10, &ExampleListener::messageCallback, this);
    }

    void ExampleListener::messageCallback(const node_example::NodeExampleData::ConstPtr &msg)
    {
      ROS_INFO("message is %s, a + b = %d", msg->message.c_str(), msg->a + msg->b);
    }
}

-----------订阅器的头文件：listener.h------
#ifndef NODE_EXAMPLE_LISTENER_H
#define NODE_EXAMPLE_LISTENER_H

#include "ros/ros.h"
#include "ros/time.h"
#include "node_example/NodeExampleData.h"
namespace node_example
{
    class ExampleListener
    {
     public:
      explicit ExampleListener(ros::NodeHandle nh);
    //message订阅后的回调函数
      void messageCallback(const node_example::NodeExampleData::ConstPtr &msg);
     private:
      ros::Subscriber sub_;
    };
}
#endif 


-----------初始化参数的设置------
有两种方法可进行初始参数的设置：
1.通过launch文件设置
<param name="a" value="1"/>
<param name="b" value="2"/>
<param name="message" value="hello"/>
<param name="rate" value="1"/>

2.通过命令行设置
rosrun node_example talker _message:="Hello world!" _a:=57 _b:=-15 _rate:=1



