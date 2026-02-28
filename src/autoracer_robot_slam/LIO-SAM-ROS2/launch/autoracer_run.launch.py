# AutoRacer LIO-SAM Launch 文件
# 仅启动 LIO-SAM 核心节点，不包含机器人 bringup 和传感器驱动
# 使用方式:
#   终端 1: ros2 launch turn_on_autoracer_robot turn_on_autoracer_robot.launch.py
#   终端 2: ros2 launch lslidar_driver lslidar_cx_launch.py
#   终端 3: ros2 launch lio_sam autoracer_run.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'autoracer_params.yaml'),
        description='LIO-SAM 参数文件路径')

    return LaunchDescription([
        params_declare,
        # map → odom 静态 TF（LIO-SAM 需要此变换作为初始值）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 1.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
        ),
        # IMU 预积分节点
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        # 点云去畸变与投影
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        # 特征提取（角点 + 平面）
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        # 全局地图优化（GTSAM 因子图）
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
        # RViz2 可视化（可选，如需单独启动 rviz2 可注释掉）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
