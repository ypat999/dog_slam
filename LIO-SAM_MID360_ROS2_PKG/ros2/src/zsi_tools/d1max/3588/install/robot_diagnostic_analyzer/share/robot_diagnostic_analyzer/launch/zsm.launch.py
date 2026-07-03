from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('robot_diagnostic_analyzer'),
        'config', 'zsm', 'robot_analyzers.yaml'
    ])

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='robot_diagnostic_analyzer',
        output='screen',
        parameters=[LaunchConfiguration('params_file', default=params_file)],
    )

    return LaunchDescription([
        aggregator,
        RegisterEventHandler(
            OnProcessExit(
                target_action=aggregator,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
