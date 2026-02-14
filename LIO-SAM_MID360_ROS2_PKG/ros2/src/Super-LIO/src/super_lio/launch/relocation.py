import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import sys, os

def generate_launch_description():
    # 首先导入全局配置

    # 正确导入global_config包
    try:
        # 方法1：通过ROS2包路径导入
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            ONLINE_LIDAR, DEFAULT_BAG_PATH, DEFAULT_RELIABILITY_OVERRIDE,
            DEFAULT_USE_SIM_TIME, MANUAL_BUILD_MAP, BUILD_TOOL, RECORD_ONLY,
            NAV2_DEFAULT_PARAMS_FILE, LIVOX_MID360_CONFIG, LIVOX_MID360_CONFIG_NO_TILT
        )
    except ImportError as e:
        print(f"方法2导入global_config失败: {e}")
        # 如果导入失败，使用默认值
        ONLINE_LIDAR = True
        DEFAULT_BAG_PATH = '/home/ztl/slam_data/livox_record_new/'
        DEFAULT_RELIABILITY_OVERRIDE = '/home/ztl/slam_data/reliability_override.yaml'
        DEFAULT_USE_SIM_TIME = False
        MANUAL_BUILD_MAP = False
        BUILD_TOOL = 'octomap_server'
        RECORD_ONLY = False
        NAV2_DEFAULT_PARAMS_FILE = '/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml'
        LIVOX_MID360_CONFIG_NO_TILT = ''

    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'relocation.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'relocation.rviz')

    use_sim_time = DEFAULT_USE_SIM_TIME
    livox_config_path = LIVOX_MID360_CONFIG_NO_TILT
    lidar_mode = "ONLINE"
    if not ONLINE_LIDAR:
        lidar_mode = "OFFLINE"

    ld = LaunchDescription()

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')
    ld.add_action(declare_rviz_arg)

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
        prefix=['taskset -c 4'],
        condition=IfCondition(PythonExpression("'" + lidar_mode + "' == 'ONLINE'"))
    )

    # 根据模式选择启动相应的节点
    ld.add_action(livox_driver_node)

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(declare_use_sim_time_arg)

    # 创建Super-LIO重定位节点
    super_lio_node = Node(
        package='super_lio',
        executable='relocation_node',
        name='relocation_node',
        output='screen',
        parameters=[config_yaml],
        prefix=['taskset -c 7'],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(super_lio_node)

    # 添加静态变换发布器
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    static_transform_odom_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_world',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'world'],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_world)

    static_transform_world_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_imu',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'imu'],
        output='screen'
    )
    ld.add_action(static_transform_world_to_imu)

    imu_to_livox_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', '0.0', '0', 'imu', 'livox_frame'],
        output='screen'
    )
    ld.add_action(imu_to_livox_frame_tf)

    livox_frame_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['-0.1', '0', '-0.1', '0', '-0.5235987756', '0', 'livox_frame', 'base_link'],
        output='screen'
    )
    ld.add_action(livox_frame_to_base_link_tf)

    base_link_to_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', '0.0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    ld.add_action(base_link_to_base_footprint_tf)

    # 根据模式添加相应的节点（按照LIO-SAM的逻辑）
    if RECORD_ONLY:
        # 仅录制模式：只启动雷达驱动
        return ld

    return ld