"""
coverage.launch.py -- the coverage executor alone.

Normally launched by minibot_bringup/clean.launch.py. Run it standalone only if
Nav2 is already up and you just want to restart the coverage run.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('minibot_coverage')
    params = os.path.join(pkg, 'config', 'coverage_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('path', description='*_coverage_path.yaml from prepare_map.py'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('resume', default_value='false'),
        Node(package='minibot_coverage', executable='coverage_server.py',
             name='coverage_server', output='screen',
             parameters=[params, {
                 'path_file': LaunchConfiguration('path'),
                 'auto_start': LaunchConfiguration('auto_start'),
                 'resume': LaunchConfiguration('resume'),
             }]),
    ])
