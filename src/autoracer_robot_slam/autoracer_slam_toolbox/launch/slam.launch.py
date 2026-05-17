"""
SLAM Toolbox 2D 建图 Launch 文件 - AutoRacer 专用

阶段 3 验收必须使用 include_bringup:=false，并按 docs/启动与运行规范.md
单独启动 phase-1 Ackermann 底盘链路和 LiDAR。include_bringup:=true 会拉起
legacy/default bringup，只允许迁移排障，不能作为阶段 3 PASS 入口。

用法:
  ros2 launch autoracer_slam_toolbox slam.launch.py include_bringup:=false   # 阶段 3 验收入口
  ros2 launch autoracer_slam_toolbox slam.launch.py use_rviz:=false
  ros2 launch autoracer_slam_toolbox slam.launch.py odom_frame:=odom_combined # 仅 legacy/EKF 排障
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('autoracer_slam_toolbox')

    # Launch 参数
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    include_bringup = LaunchConfiguration('include_bringup', default='true')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_dir, 'config', 'mapper_params_online_sync.yaml'),
    )

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
            ('cloud_in', '/point_cloud_raw'),  # C32 点云输入
            ('scan', '/scan'),                 # 2D 扫描输出
        ],
        parameters=[{
            'target_frame': 'base_link',     # 转换到 base_link 坐标系
            'transform_tolerance': 0.01,
            'min_height': 0.1,               # base_link 上方 0.1m（过滤地面）
            'max_height': 1.5,               # base_link 上方 1.5m（捕获大部分障碍物）
            'angle_min': -3.14159,           # -180°
            'angle_max': 3.14159,            # +180°（全范围扫描）
            'angle_increment': 0.00436,      # ~0.25° 分辨率
            'scan_time': 0.05,               # 20Hz 匹配 C32 发布频率
            'range_min': 0.2,                # 最小检测距离 0.2m
            'range_max': 20.0,               # 最大检测距离 20m
            'use_inf': True,                 # 无返回值用 inf 表示
            'inf_epsilon': 1.0,
        }],
    )

    # === SLAM Toolbox 节点 ===
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'odom_frame': odom_frame},
        ],
    )

    # === RViz2（可选）===
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', os.path.join(
            get_package_share_directory('autoracer_robot_urdf'),
            'rviz', 'slam.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('include_bringup', default_value='true',
                             description='是否同时启动 legacy/default 底盘驱动和 LiDAR；阶段 3 验收必须为 false'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                             description='是否启动 RViz2'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                             description='里程计坐标系；阶段 3 验收使用 odom，odom_combined 仅用于 legacy/EKF 排障'),
        DeclareLaunchArgument('slam_params_file',
                             default_value=os.path.join(pkg_dir, 'config', 'mapper_params_online_sync.yaml'),
                             description='slam_toolbox 参数文件；正式 2D 建图可切换 dense profile'),

        autoracer_robot,
        autoracer_lidar,
        pointcloud_to_laserscan_node,
        slam_toolbox_node,
        rviz_node,
    ])
