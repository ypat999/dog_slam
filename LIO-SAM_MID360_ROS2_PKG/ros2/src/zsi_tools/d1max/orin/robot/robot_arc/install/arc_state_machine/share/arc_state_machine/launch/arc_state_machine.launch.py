#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='xg',
        description='Robot type (xg, xgw, zgw, default)'
    )

    xg_config_file_arg = DeclareLaunchArgument(
        'xg_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('arc_state_machine'),
            'config', 'XG',
            'arc_state_machine_params.yaml'
        ]),
        description='Path to the configuration file'
    )

    xgw_config_file_arg = DeclareLaunchArgument(
        'xgw_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('arc_state_machine'),
            'config', 'XGW',
            'arc_state_machine_params.yaml'
        ]),
        description='Path to the configuration file'
    )

    zg_config_file_arg = DeclareLaunchArgument(
        'zg_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('arc_state_machine'),
            'config', 'ZG',
            'arc_state_machine_params.yaml'
        ]),
        description='Path to the configuration file'
    )

    zgw_config_file_arg = DeclareLaunchArgument(
        'zgw_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('arc_state_machine'),
            'config', 'ZGW',
            'arc_state_machine_params.yaml'
        ]),
        description='Path to the configuration file'
    )

    set_config_file = SetLaunchConfiguration(
        'config_file',
        PythonExpression([
            "{'xg': '", LaunchConfiguration('xg_config_file'),
            "', 'xgw': '", LaunchConfiguration('xgw_config_file'),
            "', 'zg': '", LaunchConfiguration('zgw_config_file'),
            "', 'zgw': '", LaunchConfiguration('zgw_config_file'),
            "'}.get('", LaunchConfiguration('robot_type'),
            "', '", LaunchConfiguration('xg_config_file'), "')"
        ])
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_type_log = LogInfo(
        msg=['robot_type: ', LaunchConfiguration('robot_type')]
    )

    # ARC state machine node
    arc_state_machine_node = Node(
        package='arc_state_machine',
        executable='arc_state_machine_node',
        name='arc_state_machine_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        robot_type_arg,
        xg_config_file_arg,
        xgw_config_file_arg,
        zg_config_file_arg,
        zgw_config_file_arg,
        set_config_file,
        log_level_arg,
        use_sim_time_arg,
        robot_type_log,
        arc_state_machine_node
    ])
