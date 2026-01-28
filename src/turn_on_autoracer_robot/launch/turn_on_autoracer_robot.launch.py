"""
Launch file for AutoRacer robot bringup.

This launch file starts the main chassis control node with N300 Pro IMU integration.
Reference: wheeltec_ros2/src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py

IMU 集成策略（参考 wheeltec）:
- STM32 板载 MPU6050: /imu/data_raw → /imu/data_board (remapped away, 不使用)
- N300 Pro 外置 IMU: hipnuc_imu 发布到 /imu/data_raw (高精度，实际使用)
- Madgwick 滤波器: /imu/data_raw → 滤波后用于 EKF
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    autoracer_pkg = get_package_share_directory('turn_on_autoracer_robot')

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

    # 是否使用 N300 Pro IMU (默认 true，替代 STM32 板载 MPU6050)
    use_n300pro_imu_arg = DeclareLaunchArgument(
        'use_n300pro_imu',
        default_value='true',
        description='Use N300 Pro external IMU instead of STM32 onboard MPU6050'
    )
    use_n300pro_imu = LaunchConfiguration('use_n300pro_imu')

    # IMU config for Madgwick filter
    imu_config = Path(autoracer_pkg, 'config', 'imu.yaml')

    # ========== 使用 N300 Pro IMU 时 ==========
    # STM32 node with IMU topic remapping (板载 IMU 重映射走)
    autoracer_robot_with_n300pro = Node(
        condition=IfCondition(use_n300pro_imu),
        package='turn_on_autoracer_robot',
        executable='autoracer_robot',
        name='autoracer_robot',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('usart_port_name'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        }],
        # 关键: 将 STM32 的 /imu/data_raw 重映射到 /imu/data_board
        # 这样 N300 Pro 的 hipnuc_imu 可以发布到 /imu/data_raw
        remappings=[('imu/data_raw', 'imu/data_board')]
    )

    # N300 Pro IMU driver (hipnuc_imu)
    hipnuc_imu_launch = IncludeLaunchDescription(
        condition=IfCondition(use_n300pro_imu),
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hipnuc_imu'),
                        'launch', 'imu_spec_msg.launch.py')
        )
    )

    # Madgwick IMU filter (当使用 N300 Pro 时)
    imu_filter_node = Node(
        condition=IfCondition(use_n300pro_imu),
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_config] if imu_config.exists() else [{
            'fixed_frame': 'base_footprint',
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
        }]
    )

    # ========== 不使用 N300 Pro IMU 时 (使用 STM32 板载 MPU6050) ==========
    autoracer_robot_without_n300pro = Node(
        condition=UnlessCondition(use_n300pro_imu),
        package='turn_on_autoracer_robot',
        executable='autoracer_robot',
        name='autoracer_robot',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('usart_port_name'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        }]
    )

    # ========== Static TF transforms ==========
    # base_footprint -> base_link
    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # base_footprint -> gyro_link (IMU frame)
    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'gyro_link']
    )

    return LaunchDescription([
        # Launch arguments
        usart_port_arg,
        baud_rate_arg,
        robot_frame_arg,
        odom_frame_arg,
        use_n300pro_imu_arg,
        # Robot nodes (conditional)
        autoracer_robot_with_n300pro,
        autoracer_robot_without_n300pro,
        # N300 Pro IMU (conditional)
        hipnuc_imu_launch,
        imu_filter_node,
        # Static TF
        base_to_link,
        base_to_gyro,
    ])
