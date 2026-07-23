"""
view_robot.launch.py -- inspect the URDF in RViz. No hardware needed.

    ros2 launch minibot_description view_robot.launch.py

Use this to check the TF tree, the wheel axes, and the LIDAR orientation BEFORE
you ever plug the robot in. A mirrored lidar_frame here is a mirrored map later.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('minibot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'view_robot.rviz')

    desc = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=false']),
        value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_cfg]),
    ])
