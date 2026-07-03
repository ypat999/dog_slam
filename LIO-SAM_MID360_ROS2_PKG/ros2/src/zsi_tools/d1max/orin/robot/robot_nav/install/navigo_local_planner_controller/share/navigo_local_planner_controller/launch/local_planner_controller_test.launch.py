from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("navigo_local_planner_controller"), "config", "local_planner_controller_test.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("navigo_local_planner_controller"), "config", "local_planner_controller_test.rviz"]
    )

    test_node = Node(
        package="navigo_local_planner_controller",
        executable="local_planner_controller_test_node",
        name="lpc_test_node",
        output="screen",
        parameters=[config_file],
        arguments=["--clicked_point_test"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([test_node, rviz])
