import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    try:
        global_config_path = os.path.join(
            get_package_share_directory('global_config'),
            '../../src/global_config'
        )
        sys.path.insert(0, global_config_path)
        from global_config import DEFAULT_USE_SIM_TIME, DEFAULT_NAMESPACE
    except ImportError:
        DEFAULT_USE_SIM_TIME = False
        DEFAULT_NAMESPACE = ''

    pkg_super_lio = get_package_share_directory('super_lio')

    ld = LaunchDescription()

    declare_ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='rkbot',
        description='Namespace for multi-robot support'
    )
    ld.add_action(declare_ns_arg)
    ns = LaunchConfiguration('ns', default=DEFAULT_NAMESPACE)
    ld.add_action(PushRosNamespace(ns))

    ns_base_footprint_frame = PythonExpression(
        ["'base_footprint' if '", ns, "' == '' else str('", ns, "/base_footprint')"])
    ns_base_link_frame = PythonExpression(
        ["'base_link' if '", ns, "' == '' else str('", ns, "/base_link')"])

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=str(DEFAULT_USE_SIM_TIME),
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=DEFAULT_USE_SIM_TIME)
    ld.add_action(declare_use_sim_time_arg)

    # ==================== Super-LIO Dual Node ====================
    dual_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_super_lio, 'launch', 'dual_lidar.launch.py')
        ),
        launch_arguments={
            'ns': ns,
            'use_sim_time': str(DEFAULT_USE_SIM_TIME),
        }.items(),
    )
    ld.add_action(dual_lio_launch)

    # ==================== pointcloud_to_laserscan ====================
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
            ('cloud_in', 'lio/rear/cloud_world'),
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

    # ==================== scan merger ====================
    scan_merger = Node(
        package='zg_double_lidar',
        executable='scan_merger',
        name='scan_merger',
        output='screen',
        parameters=[{'use_sim_time': DEFAULT_USE_SIM_TIME}],
        prefix=['taskset -c 4,5'],
    )
    ld.add_action(scan_merger)

    # ==================== USS republisher ====================
    pkg_share = get_package_share_directory('zg_double_lidar')
    uss_republisher = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'uss_republisher.py')],
        output='screen',
    )
    ld.add_action(uss_republisher)

    return ld
