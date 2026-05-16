"""Phase-2 point-goal tracking launch entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stage1_launch = Path(get_package_share_directory('autoracer_bringup')) / 'launch' / 'stage1_chassis.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyACM0', description='Serial port for STM32 UART4 bridge'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('counts_per_meter', default_value='0.0', description='Measured Hall counts per meter; keep 0 until calibrated'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='Start EKF for canonical /odom when /imu/data is available'),
        DeclareLaunchArgument('goal_x_m', default_value='2.0', description='Point goal x in odom frame'),
        DeclareLaunchArgument('goal_y_m', default_value='2.0', description='Point goal y in odom frame'),
        DeclareLaunchArgument('target_speed_mps', default_value='0.18', description='Low-speed tracker target speed'),
        DeclareLaunchArgument('max_target_speed_mps', default_value='0.25', description='Tracker speed clamp; override only for approved real-car tests'),
        DeclareLaunchArgument('lookahead_m', default_value='0.60', description='Pure Pursuit lookahead distance'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.05', description='Distance to goal that commands stop'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0', description='Tracker command rate'),
        DeclareLaunchArgument('prearm_zero_before_motion', default_value='true', description='Hold zero Ackermann command until chassis auto control is ready'),
        DeclareLaunchArgument('prearm_timeout_s', default_value='3.0', description='Seconds before warning while holding prearm zero command'),
        DeclareLaunchArgument('point_spacing_m', default_value='0.25', description='Generated path point spacing'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(stage1_launch)),
            launch_arguments={
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                'counts_per_meter': LaunchConfiguration('counts_per_meter'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),
        Node(
            package='autoracer_path_tracking',
            executable='goal_path_publisher',
            name='goal_path_publisher',
            output='screen',
            parameters=[{
                'goal_x_m': ParameterValue(LaunchConfiguration('goal_x_m'), value_type=float),
                'goal_y_m': ParameterValue(LaunchConfiguration('goal_y_m'), value_type=float),
                'goal_tolerance_m': ParameterValue(LaunchConfiguration('goal_tolerance_m'), value_type=float),
                'point_spacing_m': ParameterValue(LaunchConfiguration('point_spacing_m'), value_type=float),
                'frame_id': 'odom',
                'use_current_pose': True,
            }],
        ),
        Node(
            package='autoracer_path_tracking',
            executable='pure_pursuit_tracker',
            name='pure_pursuit_tracker',
            output='screen',
            parameters=[{
                'case_name': 'goal_2m_2m',
                'target_speed_mps': ParameterValue(LaunchConfiguration('target_speed_mps'), value_type=float),
                'lookahead_m': ParameterValue(LaunchConfiguration('lookahead_m'), value_type=float),
                'goal_tolerance_m': ParameterValue(LaunchConfiguration('goal_tolerance_m'), value_type=float),
                'control_rate_hz': ParameterValue(LaunchConfiguration('control_rate_hz'), value_type=float),
                'wheelbase_m': 0.60,
                'max_steering_angle_rad': 0.262,
                'max_target_speed_mps': ParameterValue(LaunchConfiguration('max_target_speed_mps'), value_type=float),
                'allow_reverse': False,
                'prearm_zero_before_motion': ParameterValue(LaunchConfiguration('prearm_zero_before_motion'), value_type=bool),
                'prearm_timeout_s': ParameterValue(LaunchConfiguration('prearm_timeout_s'), value_type=float),
            }],
        ),
    ])
