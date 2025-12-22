#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml_file(file_path):
    """Load parameters from YAML file"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def flatten_dict(d, parent_key='', sep='.'):
    """Flatten nested dictionary"""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def generate_launch_description():
    package_name = 'faster_lio'
    package_dir = get_package_share_directory(package_name)
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    # Get config file path
    config_file = os.path.join(package_dir, 'config', 'mid360.yaml')
    
    # Load parameters from YAML file
    yaml_params = load_yaml_file(config_file)
    flattened_params = flatten_dict(yaml_params)
    
    # Faster-LIO node
    fasterlio_node = Node(
        package=package_name,
        executable='run_mapping_online',
        name='fasterlio_mapping',
        output='screen',
        parameters=[flattened_params],
        arguments=[]  # Empty arguments to avoid passing params-file
    )
    
    # RVIZ node
    from launch.conditions import IfCondition
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz_cfg', 'loam_livox.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(rviz_arg)
    
    # Add the nodes
    ld.add_action(fasterlio_node)
    ld.add_action(rviz_node)
    
    return ld