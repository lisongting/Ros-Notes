---------------------------Ros C++代码理解和学习笔记2-----------------------------------


Ros的Spinning ------------------------------------------------------------------------
-----------单线程Spinning---------
ros的spinning 有两种方式：
//方式一
ros::init(argc, argv, "my_node");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe(...);
...
ros::spin();

//方式二
ros::Rate r(10); // 10 hz
while (ros::ok())
{
  ... do some work, publish some messages, etc. ...
  ros::spinOnce();
  r.sleep();
}
//上面两种方式都是在单线程中的


-----------多线程Spinning---------
//阻塞式的spinner
ros::MultiThreadedSpinner spinner(4);//使用4线程
spinner.spin();         //直到节点被关闭，否则spin()不会返回


//非阻塞式的spinner
ros::AsyncSpinner spinner(4);//使用4线程
spinner.start();
ros::waitForShutdown();
spinner.stop();//在需要关闭时调用



与发布相关的一些知识点 ------------------------------------------------------------------------
-----------进程内发布---------
//当发布服务器和订阅服务器作用在相同节点的同一个话题，roscpp可以跳过序列化/反序列化步骤，
//但只有当消息被发布为shared_ptr才会这样处理
//（shared_ptr：智能指针，会记录有多少个shared_ptrs共同指向一个对象）
//不可以修改发送后的消息，因为指针会直接传递到用户的任何进程内。如果你想发送另一条消息，你必须分配一个新的。
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name",5);
std_msgs::StringPtr = str(new std_msgs::String);
str->data = "hello world";
pub.publish(str);

//关闭所有发布器和话题
ros::shutdown();

//关闭对应的话题:
ros::Publisher::shutdown();

//带有相同类型的相同话题多次调用NodeHandle ::advertise()，在这种情况下，
//所有ros::Publisher都当做是另外一个副本

//获取发布器发布的话题
ros::Publisher::getTopic();

//Message的回调函数可以是如下三种类型之一：
* 函数
* 类的方法
* 函数对象（如boost::bind）//http://www.boost.org/doc/libs/1_37_0/libs/bind/bind.html#with_functions


-----------使用MessageEvent类---------
使用MessageEvent类可以在订阅的回调函数里获取到消息的元数据
void callback(const ros::MessageEvent<std_msgs::String const >& event ){
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();//获取连接头信息
    ros::Time receipt_time = event.getMessage();

    const std_msgs::StringConstPtr& msg = event.getMessage();
}



tf 数据类型------------------------------------------------------------------------
基本数据类型有：
1.Quaternion           tf::Quaternion
2.Vector                   tf::Vecter3
3.Point                     tf::Point
4.Pose                      tf::Pose
5.Transform             tf::Transform

//这五种Message type为：
[geometry_msgs/Quaternion]:
float64 x
float64 y
float64 z
float64 w

[geometry_msgs/Vector3]:
float64 x
float64 y
float64 z

[geometry_msgs/Point]:
float64 x
float64 y
float64 z


[geometry_msgs/Pose]:
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w


  [geometry_msgs/Transform]:
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w


tf::Stamped 模板
tf::Stamped对上述数据类型做模板化(除了tf::Transform)，并附带元素frame_id,stamp_
template <typename T>
class Stamped : public T{
    public:
        ros::Time stamp_;
        std::string frame_id_;

        Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){};

        Stamped(const T& input, const ros::Time& timestamp, const std::string & frame_id);

        void setData(const T& input);
};

tf::StampedTransform
tf::StampedTransform 是tf::Transforms的特例，它要求frame_id 、stamp 、child_frame_id.
class StampedTransform : public tf::Transform
{
public:
    //与transform联系的时间戳
  ros::Time stamp_;

  std::string frame_id_;

  std::string child_frame_id_;

  //构造函数
  StampedTransform(const tf::Transform& input, const ros::Time& timestamp,
      const std::string & frame_id, const std::string & child_frame_id):
    tf::Transform (input), stamp_ ( timestamp ), frame_id_ (frame_id), child_frame_id_(child_frame_id){ };

  StampedTransform() { };

  void setData(const tf::Transform& input){*static_cast<tf::Transform*>(this) = input;};

};

--------辅助函数----------
//返回四元组句柄
tf::Quaternion createIdentityQuaternion();

//返回从固定轴的Roll, Pitch and Yaw(滚动，俯仰和偏转)构造的tf::Quaternion四元数
tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw);

//返回从固定轴的Roll, Pitch and Yaw(滚动，俯仰和偏转)构造的geometry_msgs::Quaternion四元数
geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double roll,double pitch,double yaw);


tf 广播变换------------------------------------------------------------------------
无参构造器：
tf::TransformBroadcaster();

发送变换：
发送变换通过调用sendTransform()函数实现，传递StampedTransform
或geometry_msgs::TransformStamped为参数
如：
void sendTransform(const StampedTransform & transform);
void sendTransform(const geometry_msgs::TransformStamped & transform);


tf 使用已发布的变换------------------------------------------------------------------------
通过调用tf::TransformListener类来处理变换，它继承自tf::Transformer

构造函数：
TransformListener(const ros::NodeHandle &nh,
                             ros::Duration max_cache_time=ros::Duration(DEFAULT_CACHE_TIME),
                              bool spin_thread=true);
TransformListener(ros::Duration max_cache_time=ros::Duration(DEFAULT_CACHE_TIME), bool spin_thread=true);

辅助方法：
std::string tf::TransformListener::resolve (const std::string &frame_id)

判断是否实现变换：检查在时间time内，source_frame能否变换到target_frame
bool tf::TransformListener::canTransform (const std::string &target_frame, const std::string &source_frame,
                                                                    const ros::Time &time, std::string *error_msg=NULL)


 判断变换是否有效：检查在时间time， source_frame能否变换到target_frame
它将休眠并重试每个polling_duration，直到超时的持续时间已经过去。
 bool tf::TransformListener::waitForTransform (const std::string &target_frame, const std::string &source_frame,
                                                                        const ros::Time &time, const ros::Duration &timeout,
                                                                        const ros::Duration &polling_sleep_duration=ros::Duration(0.01),
                                                                         std::string *error_msg=NULL)


tf 异常------------------------------------------------------------------------

异常类：tf::ConnectivityException
作用：如果由于两个坐标系ID不在同一个连接的树中而无法完成请求，则抛出。

异常类：tf::ExtrapolationException
作用：如果请求的坐标系id之间存在连接，但一个或多个变换已过期，则抛出。

异常类：tf::InvalidArgument
作用：如果参数无效则抛出。 最常见的情况是非规范化的四元数。

异常类：tf::LookupException
作用：如果引用了未发布的坐标系ID，则抛出。
