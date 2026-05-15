"""Fixed phase-1 Ackermann chassis launch entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    chassis_launch = Path(get_package_share_directory('turn_on_autoracer_robot')) / 'launch' / 'ackermann_chassis.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument(
            'usart_port_name',
            default_value='/dev/ttyACM0',
            description='Serial port for STM32 UART4 bridge',
        ),
        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='115200',
            description='Serial baud rate',
        ),
        DeclareLaunchArgument(
            'counts_per_meter',
            default_value='0.0',
            description='Measured Hall counts per meter; keep 0 until calibrated',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Start EKF for canonical /odom when /imu/data is available',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(chassis_launch)),
            launch_arguments={
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                'counts_per_meter': LaunchConfiguration('counts_per_meter'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),
    ])
