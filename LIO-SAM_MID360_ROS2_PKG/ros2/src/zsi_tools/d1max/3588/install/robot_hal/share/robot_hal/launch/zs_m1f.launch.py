'''
Author: richie.li
Date: 2026-03-03 17:24:52
LastEditors: richie.li
LastEditTime: 2026-03-03 17:26:55
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.logging import get_logger
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = FindPackageShare("robot_hal")
    config_dir = PathJoinSubstitution([package_dir, "config", "zs_m1f"])

    hal_param = PathJoinSubstitution([config_dir, "robot_hal.yaml"])
    controller_param = PathJoinSubstitution(
        [config_dir, "controller_manager.yaml"]
    )
    hw_description = {'hw_description':  Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([config_dir, "hw_description.xacro"])
        ]
    )}

    robot_hal_node = Node(
        package="robot_hal",
        executable="robot_hal_node",
        parameters=[
            hal_param,
            controller_param,
            hw_description
        ],
        output="screen",
    )

    nodes = [
        robot_hal_node,
    ]

    return LaunchDescription(nodes)
