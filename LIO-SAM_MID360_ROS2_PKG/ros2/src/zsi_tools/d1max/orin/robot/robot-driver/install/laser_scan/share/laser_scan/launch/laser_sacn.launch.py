from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(get_package_share_directory(
        'laser_scan'), 'config', 'config.yaml')
    return LaunchDescription([
        Node(
            package='laser_scan',
            executable='laser_scan',
            output='screen',
            parameters=[config_file]
        )
    ])
