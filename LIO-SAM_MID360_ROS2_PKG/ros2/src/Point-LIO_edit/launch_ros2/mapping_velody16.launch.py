#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'point_lio'
    package_dir = get_package_share_directory(package_name)
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    # Get config file path for Velodyne16
    config_file = os.path.join(package_dir, 'config', 'velody16.yaml')
    
    # Point-LIO node for Velodyne16
    pointlio_node = Node(
        package=package_name,
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 1,
                'space_down_sample': True,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.4,
                'ivox_nearby_type': 6,
                'runtime_pos_log_enable': False
            }
        ],
        remappings=[
            ('/velodyne_points', '/velodyne_points'),
            ('/imu/data', '/imu/data')
        ]
    )
    
    # RVIZ node
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
    ld.add_action(pointlio_node)
    ld.add_action(rviz_node)
    
    return ld