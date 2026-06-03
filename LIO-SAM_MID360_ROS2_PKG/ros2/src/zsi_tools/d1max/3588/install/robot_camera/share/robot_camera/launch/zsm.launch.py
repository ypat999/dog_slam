from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare("robot_camera")

    param = PathJoinSubstitution(
        [package_dir, "config", "zsm.yaml"]
    )

    node = Node(
        package="robot_camera",
        executable="robot_camera_node",
        parameters=[
            param,
        ],
        output="screen",
    )

    nodes = [
        node,
    ]

    return LaunchDescription(nodes)
