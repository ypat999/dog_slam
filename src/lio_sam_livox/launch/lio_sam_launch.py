#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    lio_sam_share_dir = get_package_share_directory('lio_sam_livox')
    
    # Configuration paths
    config_file = os.path.join(lio_sam_share_dir, 'config', 'lio_sam_config.yaml')
    rviz_config_file = os.path.join(lio_sam_share_dir, 'config', 'lio_sam.rviz')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # LIO-SAM nodes
    image_projection_node = Node(
        package='lio_sam_livox',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    feature_extraction_node = Node(
        package='lio_sam_livox',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    imu_preintegration_node = Node(
        package='lio_sam_livox',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    map_optimization_node = Node(
        package='lio_sam_livox',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    transform_fusion_node = Node(
        package='lio_sam_livox',
        executable='lio_sam_transformFusion',
        name='lio_sam_transformFusion',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        image_projection_node,
        feature_extraction_node,
        imu_preintegration_node,
        map_optimization_node,
        transform_fusion_node,
        rviz_node
    ])