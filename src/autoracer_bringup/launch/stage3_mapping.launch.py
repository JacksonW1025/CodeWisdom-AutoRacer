"""Fixed phase-3 2D mapping launch entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stage1_launch = Path(get_package_share_directory('autoracer_bringup')) / 'launch' / 'stage1_chassis.launch.py'
    slam_launch = Path(get_package_share_directory('autoracer_slam_toolbox')) / 'launch' / 'slam.launch.py'
    robot_description_launch = Path(get_package_share_directory('autoracer_robot_urdf')) / 'launch' / 'robot_description.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyACM0', description='Serial port for STM32 UART4 bridge'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('counts_per_meter', default_value='0.0', description='Measured Hall counts per meter; keep 0 until calibrated'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='Start EKF for canonical /odom when /imu/data is available'),
        DeclareLaunchArgument('start_robot_description', default_value='true', description='Start URDF robot_state_publisher for base_link to sensor TF'),
        DeclareLaunchArgument('start_chassis', default_value='true', description='Start phase-1 Ackermann chassis chain'),
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start LiDAR driver for /point_cloud_raw'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz for mapping visualization'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='SLAM odom frame; phase 3 uses canonical odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_description_launch)),
            condition=IfCondition(LaunchConfiguration('start_robot_description')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(stage1_launch)),
            condition=IfCondition(LaunchConfiguration('start_chassis')),
            launch_arguments={
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                'counts_per_meter': LaunchConfiguration('counts_per_meter'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lslidar_cx_launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_lidar')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={
                'include_bringup': 'false',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'odom_frame': LaunchConfiguration('odom_frame'),
            }.items(),
        ),
    ])
