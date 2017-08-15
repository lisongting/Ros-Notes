# Ros命令 

查看有哪些环境变量：

`$ export | grep ROS` 

将setup.bash配置到环境变量中：

`source /opt/indigo/setup.bash` 

**使用rospack和rosstack**用于获取packages 和stacks的信息:

```
$ rospack find [package_name]
$ rosstack find [stack_name]
```

如果提示找不到包，需要重新source工作控件的setup.bash

`source /devel/setup.bash` 

使用**roscd**用于改变当前目录到指定的package或者stack的目录:

```
$ roscd [locationname[/subdir]]
```

使用**rosls** ，允许你直接按软件包的名称而不是绝对路径执行ls命令（罗列目录）

```
# rosls [本地包名称[/子目录]]
如：$ rosls roscpp_tutorials
```

**roscreate** ：在当前目录创建新包

```
# roscreate-pkg [package_name]
```

你能指定包的依赖：

```
roscreate-pkg [package_name] [depend1] [depend2] [depend3]
```

**rosmake** 编译Ros程序包

`rosmake [package]` 

可以利用 `rosmake`同时编译多个package

`rosmake [package1][package2] [package3]` 

catkin工作空间中的目录：

- build目录是build space的默认所在位置，同时cmake和make也是在这里被调用来配置并编译你的程序包。
- devel目录是devel space的默认所在位置, 同时也是在你安装程序包之前存放可执行文件和库文件的地方。
- src是包所存放的位置。

---

**rosnode** 指令列出活跃的节点:

`rosnode list` 

rosnode info命令返回的是关于一个特定节点的信息:

`rosnode info /rosout` 

rosrun允许你使用包名直接运行一个包内的节点(而不需要知道这个包的路径):

`rosrun [package_name] [node_name]` 

使用rqt_graph查看当前节点和话题信息：

 直接用：`rqt_graph ` 或者` rosrun rqt_graph rqt_graph` 



rostopic echo可以显示在某个话题上发布的数据:

```
rostopic echo [topic]
```

如：rostopic echo /turtle1/command_velocity

rostopic list能够列出所有当前订阅和发布的话题

`rostopic list -v` 可以显示出有关所发布和订阅的话题及其类型的详细信息

rostopic type命令用来查看所发布话题的消息类型：

`rostopic type [topic]` 

如：rostopic type /turtle1/command_velocity

使用rosmsg命令来查看消息的详细情况(数据结构)：

```
rosmsg show [Message_Name]
```

如：rosmsg show Twist

rostopic echo可以显示在某个话题上发布的数据:

`rostopic echo [topic]` 

如：rostopic echo /turtle1/command_velocity

rostopic pub可以把数据发布到当前某个正在广播的话题上。

`rostopic pub [topic] [msg_type] [args]` 

如：rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

---

roservice:

* rosservice list ：输出可用服务的信息
* rosserice call ：调用带参数的服务。使用方法：`rosservice call [service] [args]`
* rosservice type：输出服务类型
* rosservice find ：根据服务的类型寻找服务

>rosparam使得我们能够存储并操作ROS参数服务器（Parameter Server）上的数据。参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型。
>
>rosparam使用YAML标记语言的语法。
>
>一般而言，YAML的表述很自然：
>
>- 1是整型
>- 1.0是浮点型
>- one是字符串
>- true是布尔
>- [1,2,3]是整型列表
>- {a:b,c:d}是字典

启动rqt_console和rqt_logger_level

```
$ rosrun rqt_console rqt_console
$ rosrun rqt_logger_level rqt_logger_level
```

使用roslaunch,可以用来启动定义在launch文件中的多个节点：

用法：`roslaunch [package] [filename.launch]` 

使用rosrun，可以执行某些程序，如.cpp和.py等

用法：`rosrun [package] [filename.cpp/.py]`

使用如下命令查找可能出现的错误：

```
roswtf
```





---

启动gazebo仿真器

```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

在启动时如果出现`VMware: vmw_ioctl_command error Invalid argument.` 错误，则运行：

```
export SVGA_VGPU10=0
```

**运行rviz** 

```
roslaunch turtlebot_rviz_launchers view_robot.launch
```

键盘控制turtlebot移动：

```
roslaunch turtlebot_teleop keyboard_teleop.launch
```



使用gazebo默认地图：

```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

启动地图建设：

```
roslaunch turtlebot_gazebo gmapping_demo.launch
```

