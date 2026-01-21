/**
 * @file autoracer_robot.cpp
 * @brief AutoRacer 底盘串口通信节点
 *
 * 参考: wheeltec_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
 * 实现 Orin 与 STM32 的串口通信链路
 */

#include "turn_on_autoracer_robot/autoracer_robot.hpp"
#include "turn_on_autoracer_robot/Quaternion_Solution.hpp"

sensor_msgs::msg::Imu Mpu6050;  // 实例化IMU对象

/**************************************
功能: 主函数，ROS初始化
***************************************/
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    turn_on_robot Robot("autoracer_robot");
    Robot.Control();
    rclcpp::shutdown();
    return 0;
}

/**************************************
功能: IMU 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;
    transition_16 |= Data_Low;
    return transition_16;
}

/**************************************
功能: 里程计数据转换函数
***************************************/
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;   // 获取数据的高8位
    transition_16 |= Data_Low;         // 获取数据的低8位
    data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; // 速度单位从mm/s转换为m/s
    return data_return;
}

/**************************************
功能: 速度话题订阅回调函数
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist &twist_aux)
{
    short transition;  // 中间变量

    Send_Data.tx[0] = FRAME_HEADER;  // 帧头0X7B
    Send_Data.tx[1] = 0;             // 预留位
    Send_Data.tx[2] = 0;             // 预留位

    // 机器人x轴的目标线速度
    transition = 0;
    transition = twist_aux.linear.x * 1000;  // 将浮点数放大一千倍
    Send_Data.tx[4] = transition;            // 取数据的低8位
    Send_Data.tx[3] = transition >> 8;       // 取数据的高8位

    // 机器人y轴的目标线速度
    transition = 0;
    transition = twist_aux.linear.y * 1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition >> 8;

    // 机器人z轴的目标角速度
    transition = 0;
    transition = twist_aux.angular.z * 1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);  // BCC校验位
    Send_Data.tx[10] = FRAME_TAIL;                     // 帧尾0X7D

    try
    {
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
    }
}

/**************************************
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id;

    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6;
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;

    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;

    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

    imu_publisher->publish(Imu_Data_Pub);
}

/**************************************
功能: 发布里程计话题
***************************************/
void turn_on_robot::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = robot_frame_id;
    odom.twist.twist.linear.x = Robot_Vel.X;
    odom.twist.twist.linear.y = Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;

    // 根据速度选择协方差矩阵
    if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
        memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
        memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

    odom_publisher->publish(odom);
}

/**************************************
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10)
    {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }

    if (Power_voltage < 10 && Power_voltage > 0)
    {
        static rclcpp::Time Last_Time = rclcpp::Node::now();
        float time_period = (rclcpp::Node::now() - Last_Time).seconds();
        if (time_period >= 2)
        {
            cout << RED << "Robot battery: " << Power_voltage << " is too low. Need charging." << endl << RESET;
            Last_Time = rclcpp::Node::now();
        }
    }
}

/**************************************
功能: BCC校验函数
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0)  // 接收数据模式
    {
        for (k = 0; k < Count_Number; k++)
        {
            check_sum = check_sum ^ Receive_Data.rx[k];  // 按位异或
        }
    }
    if (mode == 1)  // 发送数据模式
    {
        for (k = 0; k < Count_Number; k++)
        {
            check_sum = check_sum ^ Send_Data.tx[k];  // 按位异或
        }
    }
    return check_sum;
}

/**************************************
功能: 通过串口读取并逐帧校验下位机发送过来的数据
***************************************/
bool turn_on_robot::Get_Sensor_Data()
{
    short transition_16 = 0;
    uint8_t check = 0, error = 1, Receive_Data_Pr[1];
    static int count;
    static uint8_t Last_Receive;

    Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));

    Receive_Data.rx[count] = Receive_Data_Pr[0];
    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];

    // 接受到24字节的帧头
    if ((Receive_Data_Pr[0] == FRAME_HEADER && Last_Receive == FRAME_TAIL) || count > 0)
        count++;
    else
        count = 0;

    Last_Receive = Receive_Data_Pr[0];

    if (count == 24)  // 验证数据包的长度
    {
        count = 0;
        if (Receive_Data.Frame_Tail == FRAME_TAIL)  // 验证帧尾
        {
            check = Check_Sum(22, READ_DATA_CHECK);  // BCC校验

            if (check == Receive_Data.rx[22])
            {
                error = 0;
            }
            if (error == 0)
            {
                Receive_Data.Flag_Stop = Receive_Data.rx[1];

                // 获取运动底盘速度
                Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
                Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
                Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

                // 获取IMU数据
                Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
                Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
                Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
                Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);
                Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);
                Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);

                // 加速度单位转换
                Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
                Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

                // 陀螺仪单位转换
                Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
                Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

                // 获取电池电压
                transition_16 = 0;
                transition_16 |= Receive_Data.rx[20] << 8;
                transition_16 |= Receive_Data.rx[21];
                Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;

                return true;
            }
        }
    }
    return false;
}

/**************************************
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
    rclcpp::Time current_time, last_time;
    current_time = rclcpp::Node::now();
    last_time = rclcpp::Node::now();

    while (rclcpp::ok())
    {
        current_time = rclcpp::Node::now();
        Sampling_Time = (current_time - last_time).seconds();

        if (true == Get_Sensor_Data())
        {
            // 计算位移
            Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
            Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
            Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

            // 通过IMU角速度与加速度计算姿态
            Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

            Publish_Odom();       // 发布里程计话题
            Publish_ImuSensor();  // 发布IMU话题
            Publish_Voltage();    // 发布电源电压话题

            last_time = current_time;
        }

        rclcpp::spin_some(this->get_node_base_interface());
    }
}

/**************************************
功能: 构造函数, 用于初始化
***************************************/
turn_on_robot::turn_on_robot(std::string node_name) : Node(node_name), Sampling_Time(0), Power_voltage(0)
{
    // 清空数据
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

    // 声明参数
    this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
    this->declare_parameter("serial_baud_rate", 115200);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
    this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

    // 获取参数
    this->get_parameter("usart_port_name", usart_port_name);
    this->get_parameter("serial_baud_rate", serial_baud_rate);
    this->get_parameter("odom_frame_id", odom_frame_id);
    this->get_parameter("robot_frame_id", robot_frame_id);
    this->get_parameter("gyro_frame_id", gyro_frame_id);

    // 创建发布者
    voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // 创建订阅者
    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1));

    // 尝试打开串口
    try
    {
        Stm32_Serial.setPort(usart_port_name);
        Stm32_Serial.setBaudrate(serial_baud_rate);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "autoracer_robot can not open serial port, Please check the serial port cable!");
    }

    if (Stm32_Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "autoracer_robot serial port opened successfully");
        RCLCPP_INFO(this->get_logger(), "  Port: %s", usart_port_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  Baud rate: %d", serial_baud_rate);
    }
}

/**************************************
功能: 析构函数
***************************************/
turn_on_robot::~turn_on_robot()
{
    // 发送停止运动命令
    Send_Data.tx[0] = FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;
    Send_Data.tx[3] = 0;
    Send_Data.tx[4] = 0;
    Send_Data.tx[5] = 0;
    Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
    Send_Data.tx[10] = FRAME_TAIL;

    try
    {
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
    }

    Stm32_Serial.close();
    RCLCPP_INFO(this->get_logger(), "autoracer_robot shutting down");
}
