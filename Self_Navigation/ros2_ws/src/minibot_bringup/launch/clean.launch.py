"""
clean.launch.py -- PHASE 3: coverage cleaning of a PRE-BUILT, FROZEN map.

RUNS ON THE PI. A 15-minute autonomous run must not depend on a laptop's Wi-Fi.

    map_server + AMCL     localize against the saved map. NO SLAM at runtime, so
                          nothing rewrites the map and the run is REPEATABLE.
    keepout filter        (optional) no-go zones you painted in GIMP
    nav2                  planning + live obstacle avoidance
    coverage_server       plays back the path you already generated AND LOOKED AT

BEFORE YOU RUN THIS
    1. You have maps/home.yaml + maps/home.pgm
    2. You ran, on your laptop:
         ros2 run minibot_coverage prepare_map.py --map maps/home.yaml --start 0 0
    3. YOU OPENED maps/home_coverage_preview.png AND LOOKED AT IT.
    4. You read maps/home_qa.txt and no room you care about is unreachable.

RUN
    ros2 launch minibot_bringup robot.launch.py
    ros2 launch minibot_bringup clean.launch.py \
        map:=/home/pi/ros2_ws/maps/home.yaml \
        path:=/home/pi/ros2_ws/maps/home_coverage_path.yaml

    # with keepout zones:
    #     ... use_keepout:=true mask:=/home/pi/ros2_ws/maps/home_keepout.yaml
    # to resume an interrupted run:
    #     ... resume:=true

THEN
    In RViz: "2D Pose Estimate" at the robot's real starting pose. CHECK THE
    LASER SCAN LINES UP WITH THE MAP WALLS. Only then:

    ros2 service call /coverage/start std_srvs/srv/Trigger
    ros2 topic echo /coverage_progress                      # % swept
    ros2 service call /coverage/skip std_srvs/srv/Trigger   # wedged? skip a chunk
    ros2 service call /coverage/stop std_srvs/srv/Trigger   # abort (resumable)
    ros2 service call /coverage/dock std_srvs/srv/Trigger   # go home now
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory('minibot_navigation')
    cov_pkg = get_package_share_directory('minibot_coverage')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    keepout_params = os.path.join(nav_pkg, 'config', 'keepout_params.yaml')
    cov_params = os.path.join(cov_pkg, 'config', 'coverage_params.yaml')

    # Custom behavior trees. The stock Nav2 recovery ends in Spin, which on a
    # coverage robot ABANDONS the lane it is mid-way through sweeping -- Nav2
    # replans from the new pose and carries on, leaving an uncleaned stripe that
    # nothing reports. See minibot_navigation/behavior_trees/*.xml.
    bt_dir = os.path.join(nav_pkg, 'behavior_trees')
    bt_coverage = os.path.join(bt_dir, 'navigate_through_poses_coverage.xml')
    bt_dock = os.path.join(bt_dir, 'navigate_to_pose_dock.xml')

    # nav2_bringup substitutes these into the params file at launch.
    map_yaml = LaunchConfiguration('map')
    param_substitutions = {
        'default_nav_through_poses_bt_xml': bt_coverage,
        'default_nav_to_pose_bt_xml': bt_dock,
        'yaml_filename': map_yaml,
    }
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key='', param_rewrites=param_substitutions,
        convert_types=True)

    path_yaml = LaunchConfiguration('path')
    mask_yaml = LaunchConfiguration('mask')
    use_keepout = LaunchConfiguration('use_keepout')
    auto_start = LaunchConfiguration('auto_start')
    resume = LaunchConfiguration('resume')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_yaml,
                          'params_file': configured_params,
                          'use_sim_time': 'false'}.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': configured_params,
                          'use_sim_time': 'false'}.items())

    # Two extra lifecycle nodes: a SECOND map_server publishing the keepout mask
    # as an OccupancyGrid, and the CostmapFilterInfo server that tells the costmap
    # how to interpret it. They need their own lifecycle_manager.
    keepout = GroupAction(
        condition=IfCondition(use_keepout),
        actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='keepout_filter_mask_server', output='screen',
                 parameters=[keepout_params, {'yaml_filename': mask_yaml}]),
            Node(package='nav2_map_server', executable='costmap_filter_info_server',
                 name='costmap_filter_info_server', output='screen',
                 parameters=[keepout_params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_keepout', output='screen',
                 parameters=[{'autostart': True,
                              'node_names': ['keepout_filter_mask_server',
                                             'costmap_filter_info_server']}]),
        ])

    coverage = Node(
        package='minibot_coverage', executable='coverage_server.py',
        name='coverage_server', output='screen',
        parameters=[cov_params, {
            'path_file': path_yaml,
            'auto_start': auto_start,
            'resume': resume,
        }])

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Path to the map .yaml'),
        DeclareLaunchArgument('path',
                              description='*_coverage_path.yaml from prepare_map.py'),
        DeclareLaunchArgument('mask', default_value=''),
        DeclareLaunchArgument('use_keepout', default_value='false'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Leave FALSE. Verify AMCL with a 2D Pose Estimate and '
                        'check the scan lines up with the map before letting it move.'),
        DeclareLaunchArgument('resume', default_value='false'),
        localization, navigation, keepout, coverage,
    ])
