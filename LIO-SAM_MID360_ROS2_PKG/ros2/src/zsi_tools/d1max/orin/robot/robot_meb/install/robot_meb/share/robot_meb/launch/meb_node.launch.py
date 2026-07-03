from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('robot_meb'), 'config', 'meb.yaml']
    )
    log_dir = LaunchConfiguration('log_dir')
    log_level = LaunchConfiguration('log_level')

    meb_node = Node(
        package='robot_meb',
        executable='meb_node',
        name='MEB',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        output='log'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_dir',
            default_value='/ota/alg_data/log',
            description='Directory for ROS2 log files'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error, fatal)'
        ),
        SetEnvironmentVariable('ROS_LOG_DIR', log_dir),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        meb_node
    ])
