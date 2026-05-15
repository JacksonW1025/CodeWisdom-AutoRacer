"""Fixed phase-2 path tracking launch entry point."""

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
    path_tracking_share = Path(get_package_share_directory('autoracer_path_tracking'))
    fixture_dir = path_tracking_share / 'fixtures' / 'stage2_paths'

    return LaunchDescription([
        DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyACM0', description='Serial port for STM32 UART4 bridge'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('counts_per_meter', default_value='0.0', description='Measured Hall counts per meter; keep 0 until calibrated'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='Start EKF for canonical /odom when /imu/data is available'),
        DeclareLaunchArgument('path_case', default_value='straight_2m', description='Fixture case name under fixture_dir'),
        DeclareLaunchArgument('target_speed_mps', default_value='0.20', description='Low-speed tracker target speed'),
        DeclareLaunchArgument('lookahead_m', default_value='0.60', description='Pure Pursuit lookahead distance'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.20', description='Distance to final pose that commands stop'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0', description='Tracker command rate'),
        DeclareLaunchArgument('fixture_dir', default_value=str(fixture_dir), description='Directory containing stage-2 path fixture JSON files'),
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
            executable='test_path_publisher',
            name='test_path_publisher',
            output='screen',
            parameters=[{
                'fixture_dir': LaunchConfiguration('fixture_dir'),
                'case_name': LaunchConfiguration('path_case'),
                'publish_rate_hz': 1.0,
            }],
        ),
        Node(
            package='autoracer_path_tracking',
            executable='pure_pursuit_tracker',
            name='pure_pursuit_tracker',
            output='screen',
            parameters=[{
                'case_name': LaunchConfiguration('path_case'),
                'target_speed_mps': ParameterValue(LaunchConfiguration('target_speed_mps'), value_type=float),
                'lookahead_m': ParameterValue(LaunchConfiguration('lookahead_m'), value_type=float),
                'goal_tolerance_m': ParameterValue(LaunchConfiguration('goal_tolerance_m'), value_type=float),
                'control_rate_hz': ParameterValue(LaunchConfiguration('control_rate_hz'), value_type=float),
                'wheelbase_m': 0.60,
                'max_steering_angle_rad': 0.262,
                'max_target_speed_mps': 0.25,
                'allow_reverse': False,
            }],
        ),
    ])
