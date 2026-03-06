"""
地图保存 Launch 文件 - AutoRacer 专用
在 SLAM Toolbox 建图完成后保存地图

用法:
  ros2 launch autoracer_robot_nav2 save_map.launch.py
  ros2 launch autoracer_robot_nav2 save_map.launch.py map_name:=my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('autoracer_robot_nav2')
    map_dir = os.path.join(nav_dir, 'map')
    src_map_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'map')

    map_name = LaunchConfiguration('map_name', default='autoracer_map')

    # 保存到 install 目录
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=['-f', [map_dir, '/', map_name]],
        parameters=[{
            'save_map_timeout': 20000.0,
            'free_thresh_default': 0.196,
        }],
    )

    # 备份保存到 src 目录
    map_backup = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_backup',
        output='screen',
        arguments=['-f', [src_map_dir, '/', map_name]],
        parameters=[{
            'save_map_timeout': 20000.0,
            'free_thresh_default': 0.196,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='autoracer_map',
                             description='保存的地图文件名（不含扩展名）'),
        map_saver,
        map_backup,
    ])
