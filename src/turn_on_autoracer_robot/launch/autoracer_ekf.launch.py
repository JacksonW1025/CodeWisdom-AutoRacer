"""
EKF 传感器融合 Launch 文件 (AutoRacer)

参考: reference/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/wheeltec_ekf.launch.py

融合传感器:
- odom0: 轮式里程计 (/odom) - 来自 STM32
- imu0: IMU 姿态输入 (默认 /imu/data, 可通过 launch 参数覆盖)

输出:
- /odom_combined: 融合后的里程计
- TF: odom_combined → base_footprint
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # EKF 配置文件
    ekf_config = Path(
        get_package_share_directory('turn_on_autoracer_robot'),
        'config', 'ekf.yaml'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU topic consumed by robot_localization EKF'
    )
    imu_topic = LaunchConfiguration('imu_topic')
    ekf_parameters = [ekf_config] if ekf_config.exists() else []
    ekf_parameters.append({'imu0': imu_topic})

    # EKF 节点 (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=ekf_parameters,
        # 重映射输出话题
        remappings=[('/odometry/filtered', 'odom_combined')]
    )

    return LaunchDescription([
        imu_topic_arg,
        ekf_node,
    ])
