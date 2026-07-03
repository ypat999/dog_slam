from launch import LaunchDescription
from launch.logging import get_logger
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from robot_common_utils import DEFAULT_DEVICE_JSON, DeviceInfoError, RobotType, load_device_info


def generate_launch_description():
    package_dir = FindPackageShare("robot_monitor")
    logger = get_logger("robot_monitor.launch")

    info = None
    try:
        info = load_device_info(DEFAULT_DEVICE_JSON)
        logger.info(
            f"device.json loaded: type={info.robot_type.name}, "
            f"sn={info.sn}, board={info.board}, hw_ver={info.hw_ver}, "
            f"tags={int(info.tags)}, custom={info.custom.value}"
        )
    except DeviceInfoError as e:
        logger.warning(f"failed to load device info from {DEFAULT_DEVICE_JSON}: {e}")

    if info is not None:
        if info.robot_type == RobotType.kZsm1:
            monitor_param = PathJoinSubstitution([package_dir, "config", "zsm", "robot_monitor.yaml"])
        else:
            logger.warning(f"unknown device type, load as Zsm-1.")
            monitor_param = PathJoinSubstitution([package_dir, "config", "zsm", "robot_monitor.yaml"])
    else:
        monitor_param = PathJoinSubstitution([package_dir, "config", "zsm", "robot_monitor.yaml"])

    robot_monitor_node = Node(
        package="robot_monitor",
        executable="robot_monitor_node",
        parameters=[monitor_param],
        output="screen",
    )

    return LaunchDescription([robot_monitor_node])
