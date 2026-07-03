'''
Author: richie.li
Date: 2025-12-10 22:44:29
LastEditors: richie.li
LastEditTime: 2025-12-10 22:46:38
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare("uss_driver")

    param = PathJoinSubstitution(
        [package_dir, "config", "zsm_v3.yaml"]
    )

    uss_node = Node(
        package="uss_driver",
        executable="uss_node",
        parameters=[
            param,
        ],
        output="screen",
    )

    nodes = [
        uss_node,
    ]

    return LaunchDescription(nodes)
