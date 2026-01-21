/**
 * @file autoracer_robot_node.cpp
 * @brief Placeholder node for AutoRacer chassis control
 *
 * This is a minimal placeholder node to verify the package structure.
 * TODO: Implement actual STM32/C63A serial communication
 * TODO: Implement odometry calculation and publishing
 * TODO: Implement IMU data processing
 *
 * Reference: wheeltec_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class AutoracerRobotNode : public rclcpp::Node
{
public:
    AutoracerRobotNode() : Node("autoracer_robot")
    {
        // Declare parameters (placeholders for future STM32 communication)
        this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("serial_baud_rate", 115200);
        this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

        // Get parameters
        usart_port_name_ = this->get_parameter("usart_port_name").as_string();
        serial_baud_rate_ = this->get_parameter("serial_baud_rate").as_int();

        // Publishers
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("autoracer/status", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        // Subscribers
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&AutoracerRobotNode::cmd_vel_callback, this, std::placeholders::_1));

        // Timer for periodic status publishing
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AutoracerRobotNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "AutoRacer robot node started (placeholder)");
        RCLCPP_INFO(this->get_logger(), "  Serial port: %s", usart_port_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Baud rate: %d", serial_baud_rate_);
        RCLCPP_INFO(this->get_logger(), "  TODO: Implement STM32/C63A serial communication");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // TODO: Send velocity command to STM32 via serial
        // Reference frame format from wheeltec:
        //   Header: 0x7B, 11 bytes total, Tail: 0x7D
        //   tx[3-4]: X velocity (linear.x * 1000)
        //   tx[5-6]: Y velocity (linear.y * 1000)
        //   tx[7-8]: Z angular velocity (angular.z * 1000)
        RCLCPP_DEBUG(this->get_logger(),
            "Received cmd_vel: linear.x=%.2f, angular.z=%.2f (not sent - placeholder)",
            msg->linear.x, msg->angular.z);
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "AutoRacer node running (placeholder mode)";
        status_publisher_->publish(message);

        // TODO: Read sensor data from STM32 and publish odom/imu
        // Reference receive frame from wheeltec:
        //   Header: 0x7B, 24 bytes total, Tail: 0x7D
        //   rx[2-3]: X velocity, rx[4-5]: Y velocity, rx[6-7]: Z angular velocity
        //   rx[8-19]: IMU data (accel + gyro)
        //   rx[20-21]: Battery voltage
    }

    // Parameters
    std::string usart_port_name_;
    int serial_baud_rate_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoracerRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
