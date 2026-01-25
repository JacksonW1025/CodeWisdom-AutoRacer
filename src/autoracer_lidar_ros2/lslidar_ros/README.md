### LSLIDAR_CX_V4.2.4_240410_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CX_V4.2.4_240410_ROS2为linux环境下雷达ros2驱动，程序在ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy,ubuntu 20.04 ros galactic以及ubuntu22.04 ros humble下测试通过。适用于镭神C16,C32 3.0版本和C1,C1Plus,C4,C8,C8F,CKM8,C16,MSC16,C16国产,C32W,C32WN,C32WB,C32WP,CH32R 4.0版本以及N301 5.5版本雷达

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

pcap,boost

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~

## 3.编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
colcon build
source install/setup.bash
~~~

启动单个雷达:

~~~bash
ros2 launch lslidar_driver lslidar_cx_launch.py
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py		#自动加载 rviz
~~~

启动两台雷达:

~~~bash
ros2 launch lslidar_driver lslidar_double_launch.py
ros2 launch lslidar_driver lslidar_double_rviz_launch.py	#自动加载 rviz
~~~

启动四台雷达:

~~~bash
ros2 launch lslidar_driver lslidar_four_launch.py
~~~


## 4.yaml文件参数说明：

~~~shell
/cx/lslidar_driver_node:
  ros__parameters:
    packet_rate: 1695.0
    device_ip: 192.168.1.200  #雷达ip
    msop_port: 2368           #雷达目的数据端口
    difop_port: 2369          #雷达目的设备端口
    pcl_type: false           #点云类型 true: xyzi
    add_multicast: false      #雷达是否使用组播模式 true: 使用组播模式 
    group_ip: 224.1.1.2       #组播ip
    use_time_service: false   #雷达是否使用外部授时(GPS PTP NTP) true: 使用外部授时
    min_range: 0.15           #雷达扫描最小距离(小于此值的点会被过滤)
    max_range: 200.0          #雷达扫描最大距离(大于此值的点会被过滤)
    angle_disable_min: 0      #雷达扫描最小角度裁剪 单位 0.01°  填整数
    angle_disable_max: 0      #雷达扫描最大角度裁剪 单位 0.01°  填整数
    distance_unit: 0.4
    horizontal_angle_resolution: 0.18     #10Hz:0.18  20Hz:0.36 5Hz: 0.09
    frame_id: laser_link      #点云帧id
    topic_name: lslidar_point_cloud #点云话题名称
    publish_scan: true       #是否发布 laserscan话题
    scan_num: 15              #laserscan线号
    coordinate_opt: false     #点云0度角对应方向  true: x轴正方向
    #pcap: /home/ls/work/xxx.pcap   #pcap包路径，加载pcap包时打开此注释
~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件对应的yaml文件中以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                     #组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 把录制好的pcap文件，拷贝到lslidar_ws/src/lslidar_ros/lslidar_driver/pcap文件夹下。（lslidar_ws是ros工作空间,根据实际工作空间修改）

- 修改launch文件对应的yaml文件中以下参数

  ~~~shell
  #取消注释
  pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #pcap包路径，加载pcap包时打开此注释
  ~~~



###  pcl点云类型：

- 修改launch文件对应的yaml文件中以下参数

  ~~~shell
  pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~




### 修改雷达授时方式：

source install/setup.bash

GPS授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'gps',ntp_ip: ''}"   #说明：xx为命名空间，例如cx
~~~

PTP授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ptp',ntp_ip: ''}"  #说明：xx为命名空间，例如cx  
~~~

NTP授时：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ntp',ntp_ip: '192.168.1.102'}"   #说明：xx为命名空间，例如cx
~~~



### 雷达上下电(雷达依然转动，只发设备包，不发送数据包)：

source install/setup.bash

上电：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 1}"   #说明：xx为命名空间，例如cx
~~~

下电：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 0}"   #说明：xx为命名空间，例如cx
~~~



### 雷达转动/停止转动(电机停转)：

source install/setup.bash

转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 1}"   #说明：xx为命名空间，例如cx
~~~

停止转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 0}"   #说明：xx为命名空间，例如cx
~~~



### 设置雷达转速：

source install/setup.bash

可选频率  5Hz/10Hz/20Hz

~~~bash
ros2 service call /xx/set_motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: 20}"   #说明：xx为命名空间，例如cx
~~~



### 设置雷达数据包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_msgs/srv/DataPort "{data_port: 2368}"  #范围[1025,65535]   #说明：xx为命名空间，例如cx
~~~

**备注：设置完以后，需要修改launch文件对应的yaml文件中参数，然后重启驱动程序。**



### 设置雷达设备包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_msgs/srv/DevPort "{dev_port: 2369}"   #范围[1025,65535]   #说明：xx为命名空间，例如cx
~~~

**备注：设置完以后，需要修改launch文件对应的yaml文件中参数，然后重启驱动程序。**



### 设置雷达ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_msgs/srv/DataIp "{data_ip: "192.168.1.200"}"   #说明：xx为命名空间，例如cx
~~~

**备注：设置完以后，需要修改launch文件对应的yaml文件中参数，然后重启驱动程序。**



### 设置雷达目的ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_msgs/srv/DestinationIp "{destination_ip: "192.168.1.102"}"   #说明：xx为命名空间，例如cx
~~~




## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.0.0_221031_ROS2

Modify:  original version

Date    : 2022-10-31

-----------------------



Update version : LSLIDAR_CX_V4.1.0_221227_ROS2

Modify:  1.scan话题新增强度信息

​			    2.fpga升级，C32 90度修改计算公式

​			    3.ROS驱动新增修改授时方式的功能

​			    4.新增雷达上下电,修改雷达ip，端口，转速等功能。

​			    5.新增对ros2 dashing、ros2 eloquent、ros2 humble的支持

​			    6.修复ntp授时解析问题

Date    : 2022-12-27

----------------



Update version : LSLIDAR_CX_V4.2.2_230322_ROS2

Modify:  1.增加使用时长提示

​    		   2.新增驱动版本提示

Date    : 2023-03-22

-------------------------



Update version : LSLIDAR_CX_V4.2.3_230403_ROS2

Modify:  1.fpga协议变更，修改C32W的计算公式

Date    : 2023-04-03

-------------------------



Update version : LSLIDAR_CX_V4.2.4_240410_ROS2

Modify:   1.优化代码，降低cpu占用
        		 2.支持负角度裁剪
        		 3.删除雷达型号参数，解写自动识别雷达型号
        		 4.兼容C16 C32 3.0版本，C1 C1Plus C4 C8F CKM8 C16国产版本 C32WN C32WB 4.0版本，CH32R 4.8版本，N301 5.5版本雷达
        		 5.解决3.0雷达相邻两个数据包点角度出现连续过零度问题，修复3.0雷达无法上下电
        		 6.限制雷达IP设置范围，禁止将224网段设为雷达IP
Date    : 2024-04-10
