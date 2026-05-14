"""Phase 1 Ackermann upper-computer bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('turn_on_autoracer_robot'))
    bridge_config = pkg_share / 'config' / 'ackermann_chassis.yaml'
    ekf_config = pkg_share / 'config' / 'ackermann_ekf.yaml'

    usart_port_arg = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 UART4 bridge'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'serial_baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    counts_per_meter_arg = DeclareLaunchArgument(
        'counts_per_meter',
        default_value='0.0',
        description='Hall counts per meter; must be calibrated before trusted wheel_odom'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='Start robot_localization EKF from /wheel_odom and /imu/data'
    )

    bridge = Node(
        package='turn_on_autoracer_robot',
        executable='ackermann_chassis_bridge',
        name='ackermann_chassis_bridge',
        output='screen',
        parameters=[
            bridge_config,
            {
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': ParameterValue(
                    LaunchConfiguration('serial_baud_rate'),
                    value_type=int,
                ),
                'counts_per_meter': ParameterValue(
                    LaunchConfiguration('counts_per_meter'),
                    value_type=float,
                ),
            },
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')]
    )

    ekf_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        actions=[ekf],
    )

    return LaunchDescription([
        usart_port_arg,
        baud_rate_arg,
        counts_per_meter_arg,
        use_ekf_arg,
        bridge,
        ekf_group,
    ])
