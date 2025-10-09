#!/usr/bin/env python3
"""
简单的LiDAR测试启动文件
用于验证LiDAR设备是否能正确连接并发布点云数据
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # 获取包的共享目录
    livox_slam_online_dir = get_package_share_directory('livox_slam_online')
    
    ################### user configure parameters for ros2 start ###################
    xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    default_user_config_path = os.path.join(livox_slam_online_dir, 'config', 'mid360_config.json')
    user_config_path = LaunchConfiguration('user_config_path', default=default_user_config_path)
    ################### user configure parameters for ros2 end #####################

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    
    print(f"Config file path: {default_user_config_path}")

    # 创建LiDAR驱动节点
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
    # 创建点云查看器节点（用于可视化验证）
    pointcloud_viewer_node = Node(
        package='rclcpp_components',
        executable='component_container',
        name='pointcloud_viewer_container',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # # 创建RViz2节点
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(livox_slam_online_dir, 'config', 'livox_slam.rviz')]
    # )
    
    # 创建静态变换节点，将livox_frame连接到base_link
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame']
    )
    
    # 创建DeclareLaunchArgument
    declare_user_config_path = DeclareLaunchArgument(
        'user_config_path',
        default_value=default_user_config_path,
        description='Path to Livox user config json'
    )
    
    # 返回Launch描述
    return LaunchDescription([
        declare_user_config_path,
        static_transform_publisher,
        livox_driver_node,
        # rviz_node
    ])