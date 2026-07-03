#!/usr/bin/env python3
"""
Launch file for UWB Publisher Node (C++)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('uwb_driver')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'uwb_params.yaml')
    
    # UWB Driver Node
    uwb_node = Node(
        package='uwb_driver',
        executable='uwb_driver_node',
        name='uwb_driver',
        output='screen',
        parameters=[
            config_file
        ]
    )
    
    return LaunchDescription([
        uwb_node
    ])
