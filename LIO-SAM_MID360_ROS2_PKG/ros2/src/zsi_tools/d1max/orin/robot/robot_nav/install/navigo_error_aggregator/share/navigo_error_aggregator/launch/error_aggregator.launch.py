from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_error_aggregator_params = DeclareLaunchArgument(
        'error_aggregator_params',
        default_value=PathJoinSubstitution([FindPackageShare('navigo_error_aggregator'), 'config', 'config.yaml']),
        description='Params file for navigo_error_aggregator'
    )

    load_error_aggregator_node = Node(
        package='navigo_error_aggregator',
        executable='navigo_error_aggregator_node',
        name='error_aggregator',
        parameters=[
            LaunchConfiguration('error_aggregator_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_error_aggregator_params)
    ld.add_action(load_error_aggregator_node)

    return ld
