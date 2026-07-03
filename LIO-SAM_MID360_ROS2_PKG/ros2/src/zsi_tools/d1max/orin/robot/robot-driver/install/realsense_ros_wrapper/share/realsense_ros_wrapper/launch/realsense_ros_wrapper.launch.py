from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_ros_wrapper',
            executable='realsense_ros_wrapper',
            name='realsense_ros_wrapper',
            output='screen',
            parameters=[]
        )
    ])