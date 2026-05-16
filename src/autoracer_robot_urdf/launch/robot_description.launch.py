"""
AutoRacer URDF 模型加载 Launch 文件

启动 robot_state_publisher 和 joint_state_publisher，
将 URDF 机器人模型发布到 /robot_description 参数和 TF 树。

参考: wheeltec_ros2/src/turn_on_wheeltec_robot/launch/robot_mode_description.launch.py
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_pkg = get_package_share_directory('autoracer_robot_urdf')

    # Xacro 文件路径
    xacro_file = os.path.join(urdf_pkg, 'urdf', 'autoracer.urdf.xacro')

    # 是否启动 joint_state_publisher / joint_state_publisher_gui（调试用）
    use_jsp_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='false',
        description='Launch joint_state_publisher for non-fixed joints; not required for fixed sensor TF'
    )
    use_jsp = LaunchConfiguration('use_joint_state_publisher')

    use_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui for debugging'
    )
    use_gui = LaunchConfiguration('use_joint_state_publisher_gui')

    # 直接在 launch 进程内解析 xacro，避免工作区路径含空格时 shell 命令拆分失败。
    robot_description = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher: 读取 URDF，发布 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # base_footprint 是 odom/EKF 的底盘平面坐标，base_link 是 URDF 根 link（后轴轴心高度）。
    # 阶段 3/4 的 /scan 和 SLAM/Nav2 需要 odom -> base_footprint -> base_link -> laser 连通。
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.1175',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    )

    # joint_state_publisher: 发布关节状态（默认值）
    joint_state_publisher = Node(
        condition=IfCondition(use_jsp),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # joint_state_publisher_gui: 带 GUI 的关节状态发布（调试用）
    joint_state_publisher_gui = Node(
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        use_jsp_arg,
        use_gui_arg,
        base_footprint_to_base_link,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
    ])
