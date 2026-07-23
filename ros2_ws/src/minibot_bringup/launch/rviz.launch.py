"""
rviz.launch.py -- runs on your LAPTOP. Viewing + manual override only.

Deliberately contains NO autonomy. AMCL, Nav2 and the coverage server all run on
the Pi, so if your laptop's Wi-Fi drops mid-run the robot keeps cleaning instead
of stopping dead in the hallway.

    export ROS_DOMAIN_ID=1        # same as the Pi
    ros2 launch minibot_bringup rviz.launch.py

RViz displays worth adding:
    Map          /map               the frozen map
    Map          /coverage_map      cleaned (light) vs still-to-do (dark)
    Path         /coverage_path     the planned lanes
    LaserScan    /scan
    PoseArray    /particlecloud     AMCL's belief -- WATCH THIS. If it stays
                                    spread out, AMCL has not converged and you
                                    must not start the run.
    TF, RobotModel
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('minibot_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'robot.urdf.xacro')
    rviz_cfg = os.path.join(desc_pkg, 'rviz', 'navigation.rviz')

    use_joy = LaunchConfiguration('use_joy')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': ParameterValue(
                   Command(['xacro ', xacro_file, ' use_ros2_control:=false']),
                   value_type=str)}])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                output='screen', arguments=['-d', rviz_cfg])

    joy = Node(package='joy', executable='joy_node',
               condition=IfCondition(use_joy))

    teleop = Node(package='teleop_twist_joy', executable='teleop_node',
                  name='teleop_node', condition=IfCondition(use_joy),
                  remappings=[('/cmd_vel', 'joy_vel')])

    return LaunchDescription([
        DeclareLaunchArgument('use_joy', default_value='false'),
        rsp, rviz, joy, teleop,
    ])
