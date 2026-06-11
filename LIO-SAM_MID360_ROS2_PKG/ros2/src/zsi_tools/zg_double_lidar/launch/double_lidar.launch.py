import os
import math
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    pkg_zg_double_lidar = get_package_share_directory('zg_double_lidar')

    front_config_yaml = os.path.join(pkg_zg_double_lidar, 'config', 'front_lidar.yaml')
    rear_config_yaml = os.path.join(pkg_zg_double_lidar, 'config', 'rear_lidar.yaml')

    ld = LaunchDescription()

    declare_ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='rkbot',  #DEFAULT_NAMESPACE,
        description='Namespace for multi-robot support'
    )
    ld.add_action(declare_ns_arg)
    ns = LaunchConfiguration('ns', default=DEFAULT_NAMESPACE)
    ld.add_action(PushRosNamespace(ns))

    ns_map_frame = PythonExpression(["'map' if '", ns, "' == '' else str('", ns, "/map')"])
    ns_odom_frame = PythonExpression(["'odom' if '", ns, "' == '' else str('", ns, "/odom')"])
    ns_base_footprint_frame = PythonExpression(["'base_footprint' if '", ns, "' == '' else str('", ns, "/base_footprint')"])
    ns_base_link_frame = PythonExpression(["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])
    ns_world_frame = PythonExpression(["'world' if '", ns, "' == '' else str('", ns, "/world')"])
    ns_imu_frame = PythonExpression(["'imu' if '", ns, "' == '' else str('", ns, "/imu')"])
    # ns_base_rear_frame = PythonExpression(["'base_footprint_rear' if '", ns, "' == '' else str('", ns, "/base_footprint_rear')"])
    # ns_world_rear_frame = PythonExpression(["'world_rear' if '", ns, "' == '' else str('", ns, "/world_rear')"])
    # ns_imu_rear_frame = PythonExpression(["'imu_rear' if '", ns, "' == '' else str('", ns, "/imu_rear')"])
    ns_base_link_frame = PythonExpression(["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])

    

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
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    ld.add_action(declare_use_sim_time_arg)

    front_lidar_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='front_lidar_node',
        output='screen',
        parameters=[
            front_config_yaml,
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'lio.output.tf_base_footprint_frame': ns_base_footprint_frame},
            {'lio.output.world_frame': ns_world_frame},
            {'lio.output.imu_frame': ns_imu_frame},
        ],
        prefix=['taskset -c 7'],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            # ('lio/odom', 'front_lidar/odom'),
            # ('lio/imu/odom', 'front_lidar/imu/odom'),
            # ('lio/robo/odom', 'front_lidar/robo/odom'),
            # ('lio/path', 'front_lidar/path'),
            # ('lio/cloud_world', 'front_lidar/cloud_world'),
            # ('lio/body/cloud', 'front_lidar/body/cloud'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    ld.add_action(front_lidar_node)

    rear_lidar_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='rear_lidar_node',
        output='screen',
        parameters=[
            rear_config_yaml,
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'lio.output.tf_base_footprint_frame': ns_base_footprint_frame},
            {'lio.output.world_frame': ns_world_frame},
            {'lio.output.imu_frame': ns_imu_frame},
        ],
        prefix=['taskset -c 6'],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('lio/odom', 'rear_lidar/odom'),
            ('lio/imu/odom', 'rear_lidar/imu/odom'),
            ('lio/robo/odom', 'rear_lidar/robo/odom'),
            ('lio/path', 'rear_lidar/path'),
            ('lio/cloud_world', 'rear_lidar/cloud_world'),
            ('lio/body/cloud', 'rear_lidar/body/cloud'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    ld.add_action(rear_lidar_node)

    front_pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='front_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', 'lio/cloud_world'),
            ('scan', 'scan_front'),
        ],
        parameters=[
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'target_frame': ns_base_footprint_frame},
            {'transform_tolerance': 0.1},
            {'min_height': -0.3},
            {'max_height': 1.0},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.017},
            {'scan_time': 0.1},
            {'range_min': 0.1},
            {'range_max': 200.0},
            {'use_inf': True},
            {'inf_epsilon': 1.0},
        ],
        prefix=['taskset -c 4'],
    )
    ld.add_action(front_pointcloud_to_laserscan)

    rear_pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='rear_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', 'rear_lidar/cloud_world'),
            ('scan', 'scan_rear'),
        ],
        parameters=[
            {'use_sim_time': DEFAULT_USE_SIM_TIME},
            {'target_frame': ns_base_footprint_frame},
            {'transform_tolerance': 0.1},
            {'min_height': -0.3},
            {'max_height': 1.0},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.017},
            {'scan_time': 0.1},
            {'range_min': 0.1},
            {'range_max': 200.0},
            {'use_inf': True},
            {'inf_epsilon': 1.0},
        ],
        prefix=['taskset -c 5'],
    )
    ld.add_action(rear_pointcloud_to_laserscan)

    scan_merger = Node(
        package='zg_double_lidar',
        executable='scan_merger',
        name='scan_merger',
        output='screen',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        prefix=['taskset -c 4,5'],
    )
    ld.add_action(scan_merger)

    # USS republisher - 修复Range消息的min_range/max_range并重发到/rkbot命名空间
    pkg_share = get_package_share_directory('zg_double_lidar')
    uss_republisher = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'uss_republisher.py')],
        output='screen',
    )
    ld.add_action(uss_republisher)

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

    # static_transform_odom_to_world_rear = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_odom_to_world_rear',
    #     parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #     arguments=['-0.36615', '0.0', '0.0', '0.0', str(deg_to_rad(90)), str(deg_to_rad(180)), ns_odom_frame, ns_world_rear_frame],
    #     output='screen'
    # )
    # ld.add_action(static_transform_odom_to_world_rear)

    static_transform_world_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world_to_imu',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '-0.36615', '0.0', '0.0', '0.0', ns_world_frame, ns_imu_frame],
        output='screen'
    )
    ld.add_action(static_transform_world_to_imu)

    imu_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0', '0.0', '0', str(deg_to_rad(-90)), '0', ns_imu_frame, ns_base_link_frame],
        output='screen'
    )
    ld.add_action(imu_to_base_link_tf)

    # rslidar_head -> base_link (机器人基坐标系到雷达坐标系的静态变换)
    base_link_to_rslidar_head_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_rslidar_head_tf',    
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.36615', '0', '0.0', '0.0', str(deg_to_rad(90)), '0', ns_base_link_frame, 'rslidar_head'],
        output='screen'
    )
    ld.add_action(base_link_to_rslidar_head_tf)

    # rslidar_head -> rslidar_tail (雷达到雷达的静态变换)
    rslidar_head_to_rslidar_tail_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_head_to_rslidar_tail_tf',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0', '0', '-1.05', str(deg_to_rad(180)), str(deg_to_rad(180)), str(deg_to_rad(0)), 'rslidar_head', 'rslidar_tail'],
        output='screen'
    )
    ld.add_action(rslidar_head_to_rslidar_tail_tf)

    # # rslidar_head -> rslidar_tail (雷达到雷达的静态变换)
    # rslidar_tail_to_imu_rear_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='rslidar_tail_to_imu_rear_tf',
    #     parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #     arguments=['0', '0', '0', '0', '0', '0', 'rslidar_tail', ns_imu_rear_frame],
    #     output='screen'
    # )
    # ld.add_action(rslidar_tail_to_imu_rear_tf)

    static_transform_base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_link_to_base_footprint',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_base_link_frame, ns_base_footprint_frame],
        output='screen'
    )
    ld.add_action(static_transform_base_link_to_base_footprint)

    static_transform_base_link_to_uss_left_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_link_to_uss_left_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.15', '0.10', '0.0', '1.57', '0.0', '0.0', ns_base_link_frame, 'uss_left_link'],
        output='screen'
    )
    ld.add_action(static_transform_base_link_to_uss_left_link)

    static_transform_base_link_to_uss_right_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_link_to_uss_right_link',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        arguments=['0.15', '-0.10', '0.0', '-1.57', '0.0', '0.0', ns_base_link_frame, 'uss_right_link'],
        output='screen'
    )
    ld.add_action(static_transform_base_link_to_uss_right_link)
    
    #static_transform_world_to_base_footprint = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_transform_world_to_base_footprint',
    #    parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
    #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', ns_world_frame, ns_base_frame],
    #)
    #ld.add_action(static_transform_world_to_base_footprint)

    return ld
