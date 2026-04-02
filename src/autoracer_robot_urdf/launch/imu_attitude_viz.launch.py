"""
Visualization-only IMU attitude RViz view for AutoRacer.

This launch file creates an isolated prefixed URDF tree:
  viz_world -> viz_base_link -> viz_*

The tree is only for RViz visualization and does not modify the main
robot TF chain used by SLAM, Nav2, or chassis bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_pkg = get_package_share_directory('autoracer_robot_urdf')
    xacro_file = os.path.join(urdf_pkg, 'urdf', 'autoracer.urdf.xacro')
    rviz_file = os.path.join(urdf_pkg, 'rviz', 'imu_attitude.rviz')

    imu_topic = LaunchConfiguration('imu_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    prefix = LaunchConfiguration('prefix')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str,
    )

    viz_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='viz_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[
            ('robot_description', '/viz_robot_description'),
            ('joint_states', '/viz_joint_states'),
        ],
    )

    viz_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='viz_joint_state_publisher',
        output='screen',
        remappings=[
            ('robot_description', '/viz_robot_description'),
            ('joint_states', '/viz_joint_states'),
        ],
    )

    imu_tf_broadcaster = Node(
        package='autoracer_imu_tf_broadcaster',
        executable='autoracer_imu_tf_broadcaster',
        name='viz_imu_tf_broadcaster',
        output='screen',
        parameters=[{
            'imu_topic': imu_topic,
            'parent_frame': 'viz_world',
            'child_frame': 'viz_base_link',
            'translation.x': 0.0,
            'translation.y': 0.0,
            'translation.z': 0.11,
        }],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_imu_attitude',
        arguments=['-d', rviz_file],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic used to drive the visualization-only TF tree',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='viz_',
            description='Prefix used for the visualization-only URDF tree',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz together with the visualization-only TF tree',
        ),
        viz_robot_state_publisher,
        viz_joint_state_publisher,
        imu_tf_broadcaster,
        rviz_node,
    ])
