from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_dir = FindPackageShare("robot_remote")
    remote_config_param = PathJoinSubstitution([package_dir, "config", "remote_config.yaml"])
    robot_type_config_param = PathJoinSubstitution([package_dir, "config", "zs_m1", "special_config.yaml"])

    robot_remote_lifecycle_node = LifecycleNode(
        package="robot_remote",
        executable="robot_remote",
        name="robot_remote",
        namespace="",
        output="screen",
        parameters=[remote_config_param, robot_type_config_param],
        # prefix=['xterm -e gdb -ex run --args']
        # prefix=['gnome-terminal -- gdb -ex run --args']
    )

    return LaunchDescription([
        robot_remote_lifecycle_node
    ])
