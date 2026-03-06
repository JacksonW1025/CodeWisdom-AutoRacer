"""
Nav2 自主导航 Launch 文件 - AutoRacer 专用
启动: pointcloud_to_laserscan + Nav2 (AMCL定位 + 路径规划 + MPPI控制)

前置: 需先启动底盘驱动 + LiDAR，以及已保存的地图文件

用法:
  # 加载地图进行导航（先用 SLAM 建好图并保存）
  ros2 launch autoracer_robot_nav2 navigation.launch.py map:=/path/to/map.yaml

  # SLAM 模式导航（边建图边导航）
  ros2 launch autoracer_robot_nav2 navigation.launch.py slam:=True
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    autoracer_nav_dir = get_package_share_directory('autoracer_robot_nav2')

    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='False')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(
        autoracer_nav_dir, 'map', 'autoracer_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        autoracer_nav_dir, 'param', 'autoracer_nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    log_level = LaunchConfiguration('log_level', default='info')

    # YAML 参数替换
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # === pointcloud_to_laserscan 节点 ===
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

    # === Nav2 容器 ===
    nav2_container = Node(
        condition=IfCondition(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # === Nav2 SLAM 模式 (slam=True 时) ===
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'params_file': params_file,
        }.items(),
    )

    # === Nav2 定位模式 (slam=False 时，加载地图 + AMCL) ===
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
        }.items(),
    )

    # === Nav2 导航节点 ===
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': 'nav2_container',
        }.items(),
    )

    # === RViz2 ===
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', os.path.join(autoracer_nav_dir, 'rviz', 'nav2.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='False',
                             description='是否使用 SLAM 模式（边建图边导航）'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            autoracer_nav_dir, 'map', 'autoracer_map.yaml'),
            description='地图文件路径'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            autoracer_nav_dir, 'param', 'autoracer_nav2_params.yaml'),
            description='Nav2 参数文件路径'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        pointcloud_to_laserscan_node,
        nav2_container,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
