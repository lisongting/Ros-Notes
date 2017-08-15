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
