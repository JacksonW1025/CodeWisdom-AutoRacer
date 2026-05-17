"""Save a SLAM map into an explicit test artifact directory."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    output_dir = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    if not output_dir.is_absolute():
        output_dir = Path.cwd() / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    map_name = LaunchConfiguration('map_name').perform(context)
    map_stem = output_dir / map_name

    return [Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=['-f', str(map_stem)],
        parameters=[{
            'save_map_timeout': 20000.0,
            'free_thresh_default': 0.196,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='autoracer_map',
                              description='Map file stem without extension'),
        DeclareLaunchArgument('output_dir', default_value='artifacts/maps',
                              description='Directory where map yaml/image files are written'),
        OpaqueFunction(function=launch_setup),
    ])
