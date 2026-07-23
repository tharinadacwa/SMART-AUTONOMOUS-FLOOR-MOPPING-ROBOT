"""
imu.launch.py -- the IMU chain, standalone.

    mpu6050_node        I2C -> /imu/data_raw   (no orientation)
    imu_filter_madgwick        -> /imu/data    (adds orientation, publish_tf=FALSE)

Usually launched from minibot_bringup/robot.launch.py, not directly. Run it
alone when you are debugging the IMU:

    ros2 launch minibot_imu imu.launch.py
    ros2 topic echo /imu/data_raw
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('minibot_imu')
    params = os.path.join(pkg, 'config', 'imu_params.yaml')

    return LaunchDescription([
        Node(package='minibot_imu', executable='mpu6050_node.py',
             name='mpu6050_node', output='screen', parameters=[params]),

        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', output='screen', parameters=[params],
             remappings=[('imu/data_raw', '/imu/data_raw'),
                         ('imu/data', '/imu/data')]),
    ])
