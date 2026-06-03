#!/usr/bin/env python3
"""
Launch file for IMU Publisher Node (C++)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('imu_driver')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'imu_params.yaml')
    
    # IMU Driver Node
    imu_node = Node(
        package='imu_driver',
        executable='imu_driver',
        name='imu_driver',
        output='screen',
        parameters=[
            config_file
        ]
    )
    
    return LaunchDescription([
        imu_node
    ])
