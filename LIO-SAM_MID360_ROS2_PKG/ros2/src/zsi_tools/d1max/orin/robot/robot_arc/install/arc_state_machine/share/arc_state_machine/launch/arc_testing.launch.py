#!/usr/bin/env python3

"""
Launch file for ARC State Machine testing environment.

This launch file starts:
1. ArcStateMachineNode with test subscriber enabled
2. Optionally launches the testing tool

Usage:
    # Launch only the state machine node (logs in src/arc/src/arc_state_machine/logs/arc/)
    ros2 launch arc_state_machine arc_testing.launch.py

    # Launch with testing tool
    ros2 launch arc_state_machine arc_testing.launch.py launch_test_tool:=true

    # Use custom configuration
    ros2 launch arc_state_machine arc_testing.launch.py config_file:=/path/to/config.yaml

    # Set custom log level
    ros2 launch arc_state_machine arc_testing.launch.py log_level:=debug

    # Set custom log directory
    ros2 launch arc_state_machine arc_testing.launch.py log_dir:=/home/zy/JSZR/jszr_workspace/logs
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for ARC State Machine testing."""

    # Package directory
    arc_state_machine_share = FindPackageShare('arc_state_machine')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            arc_state_machine_share, 'config', 'arc_state_machine_params.yaml'
        ]),
        description='Path to the configuration file'
    )

    fsm_config_file_arg = DeclareLaunchArgument(
        'fsm_config_file',
        default_value=PathJoinSubstitution([
            arc_state_machine_share, 'config', 'arc_state_machine.conf'
        ]),
        description='Path to the fsm configuration file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes (debug, info, warn, error, fatal)'
    )

    launch_test_tool_arg = DeclareLaunchArgument(
        'launch_test_tool',
        default_value='false',
        description='Whether to launch the testing tool automatically'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_state_monitor_arg = DeclareLaunchArgument(
        'enable_state_monitor',
        default_value='false',
        description='Enable state monitor'
    )

    # ArcStateMachineNode
    arc_state_machine_node = Node(
        package='arc_state_machine',
        executable='arc_state_machine_node',
        name='arc_state_machine_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'test.enable_state_monitor': LaunchConfiguration('enable_state_monitor'),
                    }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
        respawn=False,
        respawn_delay=2
    )

    # Testing tool (optional)
    test_tool_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                arc_state_machine_share, 'scripts', 'start_test_tool.sh'
            ]),
            '-c', LaunchConfiguration('fsm_config_file')
        ],
        name='arc_test_tool',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_test_tool')),
        emulate_tty=True
    )

    # Launch info
    launch_info = LogInfo(
        msg=[
            '\n',
            '====================================================\n',
            '  ARC State Machine Testing Environment\n',
            '====================================================\n',
            'Configuration: ', LaunchConfiguration('config_file'), '\n',
            'Log level: ', LaunchConfiguration('log_level'), '\n',
            'Test tool: ', LaunchConfiguration('launch_test_tool'), '\n',
            '\nAvailable topics:\n',
            '  /arc/arc_state         - Current state (ArcState)\n',
            '  /arc/arc_change_flag   - State transitions (ArcChangeFlag)\n',
            '  /arc/arc_change_flag_test - Test transitions (ArcChangeFlag)\n',
            '  /arc/dock_pose         - Dock pose (DockPoseStamped)\n',
            '\nTo start testing manually:\n',
            '  python3 $SHARE_DIR/scripts/arc_test_tool.py\n',
            '\nTo monitor topics:\n',
            '  ros2 topic echo /arc/arc_state\n',
            '  ros2 topic echo /arc/arc_change_flag\n',
            '====================================================\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        config_file_arg,
        fsm_config_file_arg,
        log_level_arg,
        launch_test_tool_arg,
        use_sim_time_arg,
        enable_state_monitor_arg,

        # Info
        launch_info,

        # Nodes
        arc_state_machine_node,

        # Optional testing tool
        test_tool_process,
    ])


if __name__ == '__main__':
    # This allows the launch file to be executed directly for testing
    print(__doc__)
