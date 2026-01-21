"""
Launch file for AutoRacer serial communication node only.

This launch file starts only the serial driver node without TF transforms or other components.
Reference: wheeltec_ros2/src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('turn_on_autoracer_robot')

    # Declare launch arguments
    usart_port_arg = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'serial_baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame_id',
        default_value='base_footprint',
        description='Robot base frame ID'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odometry frame ID'
    )

    gyro_frame_arg = DeclareLaunchArgument(
        'gyro_frame_id',
        default_value='gyro_link',
        description='Gyroscope frame ID'
    )

    # Serial communication node
    autoracer_robot = Node(
        package='turn_on_autoracer_robot',
        executable='autoracer_robot',
        name='autoracer_robot',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('usart_port_name'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'gyro_frame_id': LaunchConfiguration('gyro_frame_id'),
        }]
    )

    return LaunchDescription([
        usart_port_arg,
        baud_rate_arg,
        robot_frame_arg,
        odom_frame_arg,
        gyro_frame_arg,
        autoracer_robot,
    ])
