"""
GMapping 2D SLAM Launch 文件 - AutoRacer 专用
一键启动: 底盘驱动 + LiDAR + pointcloud_to_laserscan + slam_gmapping + RViz2(可选)

用法:
  ros2 launch slam_gmapping slam_gmapping.launch.py                          # 一键启动（含底盘+LiDAR）
  ros2 launch slam_gmapping slam_gmapping.launch.py include_bringup:=false   # 仅 SLAM（需自行启动底盘+LiDAR）
  ros2 launch slam_gmapping slam_gmapping.launch.py use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    include_bringup = LaunchConfiguration('include_bringup', default='true')

    # === 底盘驱动（可选）===
    bringup_dir = get_package_share_directory('turn_on_autoracer_robot')
    autoracer_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'turn_on_autoracer_robot.launch.py')),
        condition=IfCondition(include_bringup),
    )

    # === LiDAR 驱动（可选）===
    lidar_dir = get_package_share_directory('lslidar_driver')
    autoracer_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lslidar_cx_launch.py')),
        condition=IfCondition(include_bringup),
    )

    # === pointcloud_to_laserscan 节点 ===
    # 将 C32 LiDAR 的 3D 点云转换为 2D 激光扫描
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/point_cloud_raw'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.05,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    # === slam_gmapping 节点 ===
    slam_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    # === RViz2（可选）===
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_gmapping',
        arguments=['-d', os.path.join(
            get_package_share_directory('autoracer_robot_urdf'),
            'rviz', 'slam.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('include_bringup', default_value='true',
                             description='是否同时启动底盘驱动和 LiDAR（一键启动）'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                             description='是否启动 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='是否使用仿真时间'),

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        autoracer_robot,
        autoracer_lidar,
        pointcloud_to_laserscan_node,
        slam_gmapping_node,
        rviz_node,
    ])
