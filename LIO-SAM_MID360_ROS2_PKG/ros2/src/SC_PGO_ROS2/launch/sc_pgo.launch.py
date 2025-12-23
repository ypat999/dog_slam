#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    rvizscpgo_arg = DeclareLaunchArgument(
        "rvizscpgo", default_value="true", description="Launch RViz for SC-PGO"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for all nodes"
    )

    namespace = LaunchConfiguration("namespace")

    alaserPGO_node = Node(
        package="sc_pgo_ros2",
        executable="alaserPGO",
        name="alaserPGO",
        output="screen",
        parameters=[
            {"scan_line": 128},
            {"minimum_range": 0.5},
            {"mapping_line_resolution": 0.4},
            {"mapping_plane_resolution": 0.8},
            {"mapviz_filter_size": 0.05},
            {"keyframe_meter_gap": 0.5},
            {"sc_dist_thres": 0.3},
            {"sc_max_radius": 290.0},
            {"save_directory": "./save_data/"},
        ],
        remappings=[
            ("/aft_mapped_to_init", "/Odometry"),
            ("/velodyne_cloud_registered_local", "/cloud_registered_body"),
            ("/cloud_for_scancontext", "/cloud_registered_lidar"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rvizscpgo",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rvizscpgo")),
    )

    # Group nodes under namespace
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            alaserPGO_node,
            rviz_node,
        ]
    )

    return LaunchDescription([rvizscpgo_arg, namespace_arg, namespaced_group])
