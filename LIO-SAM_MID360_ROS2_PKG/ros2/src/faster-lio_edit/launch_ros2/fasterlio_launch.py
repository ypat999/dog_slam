#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import sys, os

def generate_launch_description():
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'

    # 导入全局配置
    from global_config import LIVOX_MID360_CONFIG
    
    livox_config_path = LIVOX_MID360_CONFIG
    lidar_mode = "ONLINE"
    if not ONLINE_LIDAR:
        lidar_mode = "OFFLINE"

    # Create the launch description and populate
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default=str(DEFAULT_USE_SIM_TIME))
    package_name = 'faster_lio'
    package_dir = get_package_share_directory(package_name)
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    # Get config file path
    config_file = os.path.join(package_dir, 'config', 'mid360.yaml')
    

    # 在线模式：Livox雷达驱动
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": 'livox0000000001'},
        ],
        prefix=['taskset -c 4,5'],   # 绑定 CPU 4
        condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'ONLINE'")),
    )
    ld.add_action(livox_driver_node)

    # PointCloud to LaserScan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # ('/cloud_in', '/lio_sam/deskew/cloud_deskewed'),
            ('/cloud_in', '/cloud_registered_body'),
            ('/scan', '/scan'),
        ],
        parameters=[{
            'transform_tolerance': 0.1,
            'min_height': 0.1,           # 最小高度（过滤掉地面以下的点，调整为更紧的范围）
            'max_height': 1.5,            # 最大高度（过滤掉较高的点，限制在地面附近）
            'angle_min': -3.1,        # -180度
            'angle_max': 3.1,         # 180度
            # 将角度增量精确设置为 (angle_max - angle_min) / (691 - 1)
            # 原始地图中的激光束数量为 691，实际转换产生 690 时会触发 slam_toolbox 的长度校验错误。
            # 使用精确值可以避免舍入导致的“expected 691 / got 690”问题。
            'angle_increment': 0.00869347338,
            'scan_time': 0.1,             # 扫描时间
            
            'range_min': 0.3,             # 增加最小距离，过滤掉近距离噪声 (原0.8)
            'range_max': 40.0,             # 减少最大距离，避免远距离噪声影响 (原10.0)
            'use_inf': False,              # 是否使用无穷大值（布尔类型，不使用引号）
            
            'inf_epsilon': 40.0,           # 无穷大值的替代值
            
            # # QoS设置，确保与rviz2订阅者兼容
            # 'qos_overrides./scan.publisher.reliability': 'reliable',
            # 'qos_overrides./scan.publisher.depth': 10,
            
            # 其他参数
            'use_sim_time': use_sim_time,
            # 使用当前时间戳而不是原始时间戳，避免时间戳不匹配问题
            # 'use_latest_timestamp': 'True',
            # 设置目标坐标系为odom，确保laserscan保持水平，不随baselink倾斜
            'target_frame': 'base_link',
            'concurrency_level': 1,       # 处理并发级别
        }],
        output='screen',
        prefix=['taskset -c 5'],   # 绑定 CPU 5
    )
    ld.add_action(pointcloud_to_laserscan_node)



    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    # odom -> base_link (里程计到机器人基坐标系的静态变换)
    static_transform_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_base_link)

    base_link_to_livox_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.1', '0', '0.0', '0', 'base_link', 'livox_frame'],
        output='screen'
    )
    ld.add_action(base_link_to_livox_frame_tf)


    
    # Faster-LIO node
    fasterlio_node = Node(
        package=package_name,
        executable='run_mapping_online',
        name='fasterlio_mapping',
        output='screen',
        parameters=[config_file],
        prefix=['taskset -c 7'],   # 绑定 CPU 7
    )
    
    # RVIZ node
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz_cfg', 'loam_livox.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    
    
    # Add the launch arguments
    ld.add_action(rviz_arg)
    
    # Add the nodes
    ld.add_action(fasterlio_node)
    # ld.add_action(rviz_node)
    
    return ld