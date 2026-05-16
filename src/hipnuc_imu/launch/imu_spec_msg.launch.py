##launch file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hipnuc_imu'),
        'config',
        'hipnuc_config.yaml',
    ),
    start_listener = LaunchConfiguration('start_listener')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_listener',
            default_value='false',
            description='Start the debug IMU listener node',
        ),
         Node(
            package='hipnuc_imu',
            executable='talker',
            name='IMU_publisher',
            parameters=[config],
            output='screen',
            ),
        Node(
            condition=IfCondition(start_listener),
            package='hipnuc_imu',
            executable='listener',
            output='screen'
            ),
        ])
