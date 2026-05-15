#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "autoracer_interfaces/msg/ackermann_chassis_command.hpp"
#include "autoracer_interfaces/msg/chassis_state.hpp"
#include "serial/serial.h"

namespace
{
constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr uint8_t kCommandAckermann = 0x01;
constexpr size_t kCommandSize = 11;
constexpr size_t kTelemetrySize = 24;

constexpr uint8_t kFlagEnable = 1u << 0;
constexpr uint8_t kFlagBrake = 1u << 1;
constexpr uint8_t kFlagClearFault = 1u << 2;
constexpr uint8_t kFlagEmergencyStop = 1u << 7;

constexpr uint8_t kTelemetryAutoEnabled = 1u << 0;
constexpr uint8_t kTelemetryRcOverrideActive = 1u << 1;
constexpr uint8_t kTelemetryEstopActive = 1u << 2;
constexpr uint8_t kTelemetryCommandTimeout = 1u << 3;
constexpr uint8_t kTelemetryBrakeActive = 1u << 4;

constexpr uint32_t kStatusSteeringFeedbackValid = 1u << 8;
constexpr uint32_t kStatusSteeringIsMeasured = 1u << 9;

uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t result = 0;
    for (size_t i = 0; i < length; ++i) {
        result ^= data[i];
    }
    return result;
}

int16_t read_i16(const std::vector<uint8_t> &data, size_t offset)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[offset] << 8) | static_cast<uint16_t>(data[offset + 1]));
}

uint16_t read_u16(const std::vector<uint8_t> &data, size_t offset)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[offset] << 8) | static_cast<uint16_t>(data[offset + 1]));
}

int32_t read_i32(const std::vector<uint8_t> &data, size_t offset)
{
    uint32_t raw = (static_cast<uint32_t>(data[offset]) << 24) |
                   (static_cast<uint32_t>(data[offset + 1]) << 16) |
                   (static_cast<uint32_t>(data[offset + 2]) << 8) |
                   static_cast<uint32_t>(data[offset + 3]);
    return static_cast<int32_t>(raw);
}

uint32_t read_u32(const std::vector<uint8_t> &data, size_t offset)
{
    return (static_cast<uint32_t>(data[offset]) << 24) |
           (static_cast<uint32_t>(data[offset + 1]) << 16) |
           (static_cast<uint32_t>(data[offset + 2]) << 8) |
           static_cast<uint32_t>(data[offset + 3]);
}

void write_i16(std::array<uint8_t, kCommandSize> &data, size_t offset, int16_t value)
{
    data[offset] = static_cast<uint8_t>((value >> 8) & 0xff);
    data[offset + 1] = static_cast<uint8_t>(value & 0xff);
}

int16_t to_i16_saturated(double value)
{
    const double rounded = std::round(value);
    const double clamped = std::clamp(
        rounded,
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(clamped);
}

double normalize_yaw(double yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

}  // namespace

class AckermannChassisBridge : public rclcpp::Node
{
public:
    AckermannChassisBridge()
    : Node("ackermann_chassis_bridge")
    {
        declare_parameters();
        load_parameters();
        setup_ros();
        open_serial();
    }

    ~AckermannChassisBridge() override
    {
        send_stop();
        if (serial_.isOpen()) {
            serial_.close();
        }
    }

private:
    void declare_parameters()
    {
        declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
        declare_parameter<int>("serial_baud_rate", 115200);
        declare_parameter<std::string>("odom_frame_id", "odom");
        declare_parameter<std::string>("robot_frame_id", "base_footprint");
        declare_parameter<double>("wheelbase_m", 0.60);
        declare_parameter<double>("max_steering_angle_rad", 0.393);
        declare_parameter<double>("min_turn_speed_mps", 0.05);
        declare_parameter<double>("max_auto_speed_mps", 0.50);
        declare_parameter<double>("max_reverse_speed_mps", 0.30);
        declare_parameter<double>("counts_per_meter", 0.0);
        declare_parameter<bool>("publish_power_voltage", true);
        declare_parameter<int>("serial_poll_period_ms", 5);
    }

    void load_parameters()
    {
        get_parameter("usart_port_name", usart_port_name_);
        get_parameter("serial_baud_rate", serial_baud_rate_);
        get_parameter("odom_frame_id", odom_frame_id_);
        get_parameter("robot_frame_id", robot_frame_id_);
        get_parameter("wheelbase_m", wheelbase_m_);
        get_parameter("max_steering_angle_rad", max_steering_angle_rad_);
        get_parameter("min_turn_speed_mps", min_turn_speed_mps_);
        get_parameter("max_auto_speed_mps", max_auto_speed_mps_);
        get_parameter("max_reverse_speed_mps", max_reverse_speed_mps_);
        get_parameter("counts_per_meter", counts_per_meter_);
        get_parameter("publish_power_voltage", publish_power_voltage_);
        get_parameter("serial_poll_period_ms", serial_poll_period_ms_);
    }

    void setup_ros()
    {
        ackermann_pub_ = create_publisher<autoracer_interfaces::msg::AckermannChassisCommand>(
            "ackermann_cmd", rclcpp::QoS(1).reliable());
        chassis_state_pub_ = create_publisher<autoracer_interfaces::msg::ChassisState>(
            "chassis_state", rclcpp::QoS(10).reliable());
        wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
        voltage_pub_ = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&AckermannChassisBridge::on_cmd_vel, this, std::placeholders::_1));
        ackermann_sub_ = create_subscription<autoracer_interfaces::msg::AckermannChassisCommand>(
            "ackermann_cmd",
            rclcpp::QoS(10).reliable(),
            std::bind(&AckermannChassisBridge::on_ackermann_cmd, this, std::placeholders::_1));

        const auto period = std::chrono::milliseconds(std::max(1, serial_poll_period_ms_));
        serial_timer_ = create_wall_timer(period, std::bind(&AckermannChassisBridge::read_serial, this));
    }

    void open_serial()
    {
        try {
            serial_.setPort(usart_port_name_);
            serial_.setBaudrate(static_cast<uint32_t>(serial_baud_rate_));
            serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (const serial::IOException &) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", usart_port_name_.c_str());
            return;
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(
                get_logger(),
                "Phase 1 Ackermann chassis bridge opened %s at %d",
                usart_port_name_.c_str(),
                serial_baud_rate_);
        }
    }

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        autoracer_interfaces::msg::AckermannChassisCommand command;
        command.header.stamp = now();
        command.speed_mps = static_cast<float>(msg->linear.x);
        command.enable = true;
        command.brake = false;
        command.clear_fault = false;
        command.emergency_stop = false;

        if (std::abs(msg->linear.x) >= min_turn_speed_mps_) {
            command.steering_angle_rad = static_cast<float>(
                std::atan(wheelbase_m_ * msg->angular.z / msg->linear.x));
        } else {
            command.speed_mps = 0.0f;
            command.steering_angle_rad = 0.0f;
        }

        ackermann_pub_->publish(command);
    }

    void on_ackermann_cmd(const autoracer_interfaces::msg::AckermannChassisCommand::SharedPtr msg)
    {
        send_command(*msg);
    }

    void send_command(const autoracer_interfaces::msg::AckermannChassisCommand &command)
    {
        double speed_mps = std::clamp(
            static_cast<double>(command.speed_mps),
            -max_reverse_speed_mps_,
            max_auto_speed_mps_);
        const double steering_angle_rad = std::clamp(
            static_cast<double>(command.steering_angle_rad),
            -max_steering_angle_rad_,
            max_steering_angle_rad_);

        uint8_t flags = 0;
        if (command.enable) {
            flags |= kFlagEnable;
        }
        if (command.brake) {
            flags |= kFlagBrake;
            speed_mps = 0.0;
        }
        if (command.clear_fault) {
            flags |= kFlagClearFault;
        }
        if (command.emergency_stop) {
            flags |= kFlagEmergencyStop;
            speed_mps = 0.0;
        }
        if (!command.enable) {
            speed_mps = 0.0;
        }

        std::array<uint8_t, kCommandSize> frame{};
        frame[0] = kFrameHeader;
        frame[1] = kCommandAckermann;
        frame[2] = flags;
        write_i16(frame, 3, to_i16_saturated(speed_mps * 1000.0));
        write_i16(frame, 5, to_i16_saturated(steering_angle_rad * 1000.0));
        write_i16(frame, 7, 0);
        frame[9] = checksum(frame.data(), 9);
        frame[10] = kFrameTail;

        write_serial(frame);
    }

    void send_stop()
    {
        autoracer_interfaces::msg::AckermannChassisCommand command;
        command.enable = false;
        command.brake = true;
        command.clear_fault = false;
        command.emergency_stop = false;
        command.speed_mps = 0.0f;
        command.steering_angle_rad = 0.0f;
        send_command(command);
    }

    void write_serial(const std::array<uint8_t, kCommandSize> &frame)
    {
        if (!serial_.isOpen()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial port is not open");
            return;
        }

        try {
            serial_.write(frame.data(), frame.size());
        } catch (const serial::IOException &) {
            RCLCPP_ERROR(get_logger(), "Failed to write Ackermann command");
        }
    }

    void read_serial()
    {
        if (!serial_.isOpen()) {
            return;
        }

        try {
            const size_t available = std::min<size_t>(serial_.available(), 256);
            if (available == 0) {
                return;
            }
            std::vector<uint8_t> data;
            const size_t read_count = serial_.read(data, available);
            for (size_t i = 0; i < read_count; ++i) {
                push_rx_byte(data[i]);
            }
        } catch (const serial::IOException &) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read serial data");
        }
    }

    void push_rx_byte(uint8_t byte)
    {
        if (rx_frame_.empty() && byte != kFrameHeader) {
            return;
        }

        rx_frame_.push_back(byte);
        if (rx_frame_.size() == kTelemetrySize) {
            process_telemetry(rx_frame_);
            rx_frame_.clear();
        }
    }

    void process_telemetry(const std::vector<uint8_t> &frame)
    {
        if (frame[0] != kFrameHeader || frame[23] != kFrameTail) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected malformed telemetry frame");
            return;
        }
        if (checksum(frame.data(), 22) != frame[22]) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected telemetry frame with bad BCC");
            return;
        }
        const uint8_t status_flags = frame[1];
        const uint8_t seq = frame[2];
        const int32_t hall_delta_count = read_i32(frame, 3);
        const int16_t speed_mmps = read_i16(frame, 7);
        const int16_t steering_mrad = read_i16(frame, 9);
        const int16_t yaw_rate_mradps = read_i16(frame, 11);
        const uint16_t battery_mv = read_u16(frame, 13);
        const uint32_t status_bits = read_u32(frame, 17);

        autoracer_interfaces::msg::ChassisState state;
        state.header.stamp = now();
        state.header.frame_id = robot_frame_id_;
        state.seq = seq;
        state.hall_delta_count = hall_delta_count;
        state.delta_s_m = counts_per_meter_ > 0.0
            ? static_cast<float>(hall_delta_count / counts_per_meter_)
            : 0.0f;
        state.actual_speed_mps = static_cast<float>(speed_mmps / 1000.0);
        state.steering_angle_rad = static_cast<float>(steering_mrad / 1000.0);
        state.steering_feedback_valid = (status_bits & kStatusSteeringFeedbackValid) != 0;
        state.steering_is_measured = (status_bits & kStatusSteeringIsMeasured) != 0;
        state.estimated_yaw_rate_radps = static_cast<float>(yaw_rate_mradps / 1000.0);
        state.battery_voltage = static_cast<float>(battery_mv / 1000.0);
        state.auto_enabled = (status_flags & kTelemetryAutoEnabled) != 0;
        state.rc_override_active = (status_flags & kTelemetryRcOverrideActive) != 0;
        state.estop_active = (status_flags & kTelemetryEstopActive) != 0;
        state.brake_active = (status_flags & kTelemetryBrakeActive) != 0;
        state.command_timeout = (status_flags & kTelemetryCommandTimeout) != 0;
        state.status_bits = status_bits;

        chassis_state_pub_->publish(state);
        publish_voltage(state.battery_voltage);
        publish_wheel_odom(state);
    }

    void publish_voltage(float voltage)
    {
        if (!publish_power_voltage_) {
            return;
        }
        std_msgs::msg::Float32 msg;
        msg.data = voltage;
        voltage_pub_->publish(msg);
    }

    void publish_wheel_odom(const autoracer_interfaces::msg::ChassisState &state)
    {
        if (counts_per_meter_ <= 0.0) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "counts_per_meter must be > 0 before publishing wheel_odom");
            return;
        }
        if (!state.steering_feedback_valid) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Skipping wheel_odom because steering feedback is invalid");
            return;
        }

        const double delta_s = state.hall_delta_count / counts_per_meter_;
        const double delta_yaw = delta_s * std::tan(state.steering_angle_rad) / wheelbase_m_;
        x_m_ += delta_s * std::cos(yaw_rad_ + delta_yaw * 0.5);
        y_m_ += delta_s * std::sin(yaw_rad_ + delta_yaw * 0.5);
        yaw_rad_ = normalize_yaw(yaw_rad_ + delta_yaw);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = state.header.stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = robot_frame_id_;
        odom.pose.pose.position.x = x_m_;
        odom.pose.pose.position.y = y_m_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = state.actual_speed_mps;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = state.estimated_yaw_rate_radps;

        fill_covariance(odom, state.steering_is_measured);
        wheel_odom_pub_->publish(odom);
    }

    void fill_covariance(nav_msgs::msg::Odometry &odom, bool steering_is_measured)
    {
        std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
        std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);

        const double yaw_stddev = steering_is_measured ? 0.20 : 0.40;
        const double yaw_rate_stddev = steering_is_measured ? 0.20 : 0.40;

        odom.pose.covariance[0] = 0.05 * 0.05;
        odom.pose.covariance[7] = 0.10 * 0.10;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = yaw_stddev * yaw_stddev;

        odom.twist.covariance[0] = 0.05 * 0.05;
        odom.twist.covariance[7] = 1.00;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = yaw_rate_stddev * yaw_rate_stddev;
    }

    serial::Serial serial_;
    std::vector<uint8_t> rx_frame_;

    rclcpp::Publisher<autoracer_interfaces::msg::AckermannChassisCommand>::SharedPtr ackermann_pub_;
    rclcpp::Publisher<autoracer_interfaces::msg::ChassisState>::SharedPtr chassis_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<autoracer_interfaces::msg::AckermannChassisCommand>::SharedPtr ackermann_sub_;
    rclcpp::TimerBase::SharedPtr serial_timer_;

    std::string usart_port_name_;
    std::string odom_frame_id_;
    std::string robot_frame_id_;
    int serial_baud_rate_ = 115200;
    int serial_poll_period_ms_ = 5;
    bool publish_power_voltage_ = true;
    double wheelbase_m_ = 0.60;
    double max_steering_angle_rad_ = 0.393;
    double min_turn_speed_mps_ = 0.05;
    double max_auto_speed_mps_ = 0.50;
    double max_reverse_speed_mps_ = 0.30;
    double counts_per_meter_ = 0.0;
    double x_m_ = 0.0;
    double y_m_ = 0.0;
    double yaw_rad_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannChassisBridge>());
    rclcpp::shutdown();
    return 0;
}
