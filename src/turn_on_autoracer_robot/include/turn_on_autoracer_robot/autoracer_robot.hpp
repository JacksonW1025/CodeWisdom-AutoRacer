#ifndef __AUTORACER_ROBOT_H_
#define __AUTORACER_ROBOT_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "serial/serial.h"

#include "autoracer_interfaces/msg/supersonic.hpp"
#include "autoracer_interfaces/srv/set_rgb.hpp"

using namespace std;

// 终端颜色输出
#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

// 宏定义 - 与 wheeltec 保持一致
#define SEND_DATA_CHECK   1          // 发送数据校验标志位
#define READ_DATA_CHECK   0          // 接收数据校验标志位
#define FRAME_HEADER      0X7B       // 帧头
#define FRAME_TAIL        0X7D       // 帧尾
#define RECEIVE_DATA_SIZE 24         // 下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         // ROS向下位机发送的数据的长度
#define PI                3.1415926f // 圆周率

// 陀螺仪转换系数：与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
// 陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
// 加速度计转换系数：与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::msg::Imu Mpu6050; // 外部变量，IMU话题数据

// 协方差矩阵，用于里程计话题数据
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9} ;

// 速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
}Vel_Pos_Data;

// IMU数据结构体
typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;
}MPU6050_DATA;

// ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE];
    float X_speed;
    float Y_speed;
    float Z_speed;
    unsigned char Frame_Tail;
}SEND_DATA;

// 下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;
    float Y_speed;
    float Z_speed;
    float Power_Voltage;
    unsigned char Frame_Tail;
}RECEIVE_DATA;

// 机器人底盘类
class turn_on_robot : public rclcpp::Node
{
public:
    turn_on_robot(std::string node_name);  // 构造函数
    ~turn_on_robot();                       // 析构函数
    void Control();                         // 循环控制代码
    serial::Serial Stm32_Serial;            // 串口对象

private:
    float Sampling_Time;  // 采样时间，用于积分求位移

    // 速度话题订阅回调函数
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist &twist_aux);

    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;

    void Publish_Odom();      // 发布里程计话题
    void Publish_ImuSensor(); // 发布IMU传感器话题
    void Publish_Voltage();   // 发布电源电压话题

    // 从串口读取运动底盘速度、IMU、电源电压数据
    bool Get_Sensor_Data();

    // BCC校验函数
    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);

    // IMU数据转换读取
    short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);

    // 里程计数据转化读取
    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

    // 参数变量
    string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
    int serial_baud_rate;

    // 数据结构体
    RECEIVE_DATA Receive_Data;  // 串口接收数据结构体
    SEND_DATA Send_Data;        // 串口发送数据结构体
    Vel_Pos_Data Robot_Pos;     // 机器人的位置
    Vel_Pos_Data Robot_Vel;     // 机器人的速度
    MPU6050_DATA Mpu6050_Data;  // IMU数据
    float Power_voltage;        // 电源电压
};

#endif
