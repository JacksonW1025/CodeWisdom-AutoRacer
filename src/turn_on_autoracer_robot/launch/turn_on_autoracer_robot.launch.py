"""
Launch file for AutoRacer robot bringup.

This launch file starts the main chassis control node.
Reference: wheeltec_ros2/src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    usart_port_arg = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32/C63A communication'
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

    # Main robot node
    autoracer_robot_node = Node(
        package='turn_on_autoracer_robot',
        executable='autoracer_robot_node',
        name='autoracer_robot',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('usart_port_name'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        }]
    )

    # Static transform: base_footprint -> base_link
    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Static transform: base_footprint -> gyro_link
    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'gyro_link']
    )

    return LaunchDescription([
        usart_port_arg,
        baud_rate_arg,
        robot_frame_arg,
        odom_frame_arg,
        autoracer_robot_node,
        base_to_link,
        base_to_gyro,
    ])
