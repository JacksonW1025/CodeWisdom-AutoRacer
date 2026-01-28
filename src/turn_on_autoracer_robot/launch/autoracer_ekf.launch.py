"""
EKF 传感器融合 Launch 文件 (AutoRacer)

参考: reference/wheeltec_ros2/src/turn_on_wheeltec_robot/launch/wheeltec_ekf.launch.py

融合传感器:
- odom0: 轮式里程计 (/odom) - 来自 STM32
- imu0: N300 Pro IMU (/imu/data_raw) - 来自 hipnuc_imu

输出:
- /odom_combined: 融合后的里程计
- TF: odom_combined → base_footprint
"""

from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # EKF 配置文件
    ekf_config = Path(
        get_package_share_directory('turn_on_autoracer_robot'),
        'config', 'ekf.yaml'
    )

    # EKF 节点 (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        # 重映射输出话题
        remappings=[('/odometry/filtered', 'odom_combined')]
    )

    return LaunchDescription([
        ekf_node,
    ])
