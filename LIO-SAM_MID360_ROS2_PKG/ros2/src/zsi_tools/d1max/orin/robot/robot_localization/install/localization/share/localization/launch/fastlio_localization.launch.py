import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    parameter_config = os.path.join(get_package_share_directory(
        'localization'), 'config', 'ros_params_fastlio_loc.yaml')
    return LaunchDescription([
        Node(
            package='localization',
            executable='localization',
            parameters=[
                parameter_config
            ],
            arguments=['--ros-args', '--log-level', 'localization:=DEBUG'],
            output='screen'
        )
    ])
