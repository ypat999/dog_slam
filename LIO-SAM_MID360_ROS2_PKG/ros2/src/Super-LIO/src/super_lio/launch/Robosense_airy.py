import os
import math
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def deg_to_rad(degrees):
    return degrees * math.pi / 180.0


def generate_launch_description():
    try:
        global_config_path = os.path.join(
            get_package_share_directory('global_config'),
            '../../src/global_config'
        )
        sys.path.insert(0, global_config_path)
        from global_config import (
            DEFAULT_USE_SIM_TIME, DEFAULT_NAMESPACE
        )
    except ImportError:
        DEFAULT_USE_SIM_TIME = False
        DEFAULT_NAMESPACE = ''

    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'robosense_airy.yaml')

    ld = LaunchDescription()

    declare_ns_arg = DeclareLaunchArgument(
        'ns',
        default_value=DEFAULT_NAMESPACE,
        description='Namespace for multi-robot support'
    )
    ns = LaunchConfiguration('ns')

    ns_map_frame = PythonExpression(["'map' if '", ns, "' == '' else str('", ns, "/map')"])
    ns_odom_frame = PythonExpression(["'odom' if '", ns, "' == '' else str('", ns, "/odom')"])
    ns_base_frame = PythonExpression(["'base_footprint' if '", ns, "' == '' else str('", ns, "/base_footprint')"])
    ns_world_frame = PythonExpression(["'world' if '", ns, "' == '' else str('", ns, "/world')"])
    ns_imu_frame = PythonExpression(["'imu' if '", ns, "' == '' else str('", ns, "/imu')"])
    ns_base_link_frame = PythonExpression(["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])

    ld.add_action(declare_ns_arg)

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')
    ld.add_action(declare_rviz_arg)

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=str(DEFAULT_USE_SIM_TIME),
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(declare_use_sim_time_arg)

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[
            config_yaml,
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'lio.output.tf_base_footprint_frame': ns_base_frame},
            {'lio.output.world_frame': ns_world_frame},
            {'lio.output.imu_frame': ns_imu_frame},
        ],
        prefix=['taskset -c 7'],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/lio/odom', 'lio/odom'),
            ('/lio/imu/odom', 'lio/imu/odom'),
            ('/lio/robo/odom', 'lio/robo/odom'),
            ('/lio/path', 'lio/path'),
            ('/lio/cloud_world', 'lio/cloud_world'),
            ('/lio/body/cloud', 'lio/body/cloud'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    ld.add_action(super_lio_node)

    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_to_odom',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_map_frame, ns_odom_frame],
        output='screen'
    )
    ld.add_action(static_transform_map_to_odom)

    static_transform_odom_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_world',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.36615', '0.0', '0.0', '0.0', str(deg_to_rad(90)), '0.0', ns_odom_frame, ns_world_frame],
        output='screen'
    )
    ld.add_action(static_transform_odom_to_world)

    static_transform_world_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_imu',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_world_frame, ns_imu_frame],
        output='screen'
    )
    ld.add_action(static_transform_world_to_imu)

    imu_to_rslidar_head_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_rslidar_head_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', '0.0', '0', ns_imu_frame, 'rslidar_head'],
        output='screen'
    )
    ld.add_action(imu_to_rslidar_head_tf)

    # rslidar_head -> base_link (机器人基坐标系到雷达坐标系的静态变换)
    rslidar_head_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_head_to_base_link_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0', '0', '-0.36615', '0.0', str(deg_to_rad(-90)), '0', 'rslidar_head', ns_base_link_frame],
        output='screen'
    )
    ld.add_action(rslidar_head_to_base_link_tf)

    # rslidar_head -> rslidar_tail (雷达到雷达的静态变换)
    rslidar_head_to_rslidar_tail_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_head_to_rslidar_tail_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0', '0', '-0.7323', str(deg_to_rad(180)), str(deg_to_rad(180)), str(deg_to_rad(0)), 'rslidar_head', 'rslidar_tail'],
        output='screen'
    )
    ld.add_action(rslidar_head_to_rslidar_tail_tf)

    static_transform_world_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_base_footprint',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_world_frame, ns_base_frame],
        output='screen'
    )
    # ld.add_action(static_transform_world_to_base_footprint)

    return ld
