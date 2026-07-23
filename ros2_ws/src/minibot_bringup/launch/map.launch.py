"""
map.launch.py -- PHASE 1: build the map. Run ONCE.

    slam_toolbox (mapping) -> /map, map->odom

    ros2 launch minibot_bringup robot.launch.py     # terminal 1
    ros2 launch minibot_bringup map.launch.py       # terminal 2
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/joy_vel            # terminal 3, drive it

DRIVE SLOWLY, AND CLOSE THE LOOP -- return to where you started before saving.
Without a loop closure slam_toolbox has no chance to correct accumulated drift,
and your map ends up subtly skewed. Every coverage lane then inherits that skew.

Save:
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/home
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_pkg = get_package_share_directory('minibot_navigation')
    slam_params = os.path.join(nav_pkg, 'config', 'slam_mapping.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')]),
        launch_arguments={'slam_params_file': slam_params,
                          'use_sim_time': 'false'}.items())

    return LaunchDescription([slam])
