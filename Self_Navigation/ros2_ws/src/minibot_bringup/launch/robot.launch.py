"""
robot.launch.py -- THE HARDWARE LAYER. Runs on the Pi. Always running.

WHAT STARTS, AND WHAT DEPENDS ON WHAT
    robot_state_publisher     URDF -> static TF (base_link, lidar_frame, imu_link)
      -> ros2_control_node    loads minibot_hardware/StepperDiffDrive, opens the
                              STM32 serial port
        -> joint_state_broadcaster
          -> diff_drive_controller
                 SUB /diff_drive_controller/cmd_vel (TwistStamped)
                 PUB /diff_drive_controller/odom    (NO TF -- see below)

    sllidar_ros2              RPLIDAR C1 -> /scan   (frame: lidar_frame)
    mpu6050_node              MPU6050    -> /imu/data_raw
    imu_filter_madgwick       -> /imu/data           (publish_tf: FALSE)
    ekf_filter_node           wheel odom + IMU -> /odom AND odom->base_link TF
                              <<< THE SOLE PUBLISHER OF THAT TF >>>

    twist_mux                 joy_vel (prio 100) beats nav_vel (prio 10)
                                  -> /diff_drive_controller/cmd_vel
    twist_stamper             /cmd_vel_smoothed (Twist) -> /nav_vel (TwistStamped)

    diagnostic_aggregator     /diagnostics -> a tree for rqt_robot_monitor

map -> odom is NOT produced here. AMCL (clean.launch.py) or slam_toolbox
(map.launch.py) owns that edge.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('minibot_description')
    bring_pkg = get_package_share_directory('minibot_bringup')
    imu_pkg = get_package_share_directory('minibot_imu')

    use_imu = LaunchConfiguration('use_imu')
    lidar_port = LaunchConfiguration('lidar_port')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'robot.urdf.xacro')
    controllers = os.path.join(bring_pkg, 'config', 'controllers.yaml')
    twist_mux_cfg = os.path.join(bring_pkg, 'config', 'twist_mux.yaml')
    diag_cfg = os.path.join(bring_pkg, 'config', 'diagnostics.yaml')
    imu_params = os.path.join(imu_pkg, 'config', 'imu_params.yaml')
    ekf_cfg = os.path.join(imu_pkg, 'config', 'ekf.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str),
        'use_sim_time': False,
    }

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[robot_description])

    ros2_control = Node(package='controller_manager', executable='ros2_control_node',
                        parameters=[robot_description, controllers], output='screen')

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'])

    ddc = Node(package='controller_manager', executable='spawner',
               arguments=['diff_drive_controller', '--param-file', controllers])

    mux = Node(package='twist_mux', executable='twist_mux', name='twist_mux',
               output='screen',
               parameters=[twist_mux_cfg, {'use_stamped': True}],
               remappings=[('cmd_vel_out', 'diff_drive_controller/cmd_vel')])

    stamper = Node(package='twist_stamper', executable='twist_stamper',
                   name='twist_stamper', output='screen',
                   remappings=[('cmd_vel_in', 'cmd_vel_smoothed'),
                               ('cmd_vel_out', 'nav_vel')])

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'launch', 'sllidar_c1_launch.py')]),
        launch_arguments={'serial_port': lidar_port,
                          'frame_id': 'lidar_frame'}.items())

    imu = Node(package='minibot_imu', executable='mpu6050_node.py',
               name='mpu6050_node', output='screen',
               condition=IfCondition(use_imu), parameters=[imu_params])

    madgwick = Node(package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node', name='imu_filter',
                    output='screen', condition=IfCondition(use_imu),
                    parameters=[imu_params],
                    remappings=[('imu/data_raw', '/imu/data_raw'),
                                ('imu/data', '/imu/data')])

    ekf = Node(package='robot_localization', executable='ekf_node',
               name='ekf_filter_node', output='screen',
               condition=IfCondition(use_imu), parameters=[ekf_cfg],
               remappings=[('odometry/filtered', '/odom')])

    diag = Node(package='diagnostic_aggregator', executable='aggregator_node',
                name='diagnostic_aggregator', output='screen',
                parameters=[diag_cfg])

    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/minibot_lidar'),

        rsp,
        # Strict ordering. ros2_control cannot load the plugin until
        # robot_description exists; the spawners cannot run until
        # controller_manager is up.
        RegisterEventHandler(OnProcessStart(target_action=rsp, on_start=[ros2_control])),
        RegisterEventHandler(OnProcessStart(target_action=ros2_control, on_start=[jsb])),
        RegisterEventHandler(OnProcessStart(target_action=jsb, on_start=[ddc])),

        mux, stamper, lidar, imu, madgwick, ekf, diag,
    ])
