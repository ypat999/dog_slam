from launch import LaunchDescription
from launch.logging import get_logger
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from robot_common_utils import DEFAULT_DEVICE_JSON, DeviceInfoError, RobotType, load_device_info

def generate_launch_description():
    package_dir = FindPackageShare("robot_roamerx")
    logger = get_logger("robot_roamerx.launch")
    param_file = LaunchConfiguration("param_file")
    device_type = 0

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
        device_type = int(info.robot_type);


    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_roamerx'),
                'config/zsm',
                'robot_roamerx.yaml',
            ]),
            description='Path to node parameter file',
        ),
        Node(
            package='robot_roamerx',
            executable='robot_roamerx_node',
            name='robot_roamerx',
            output='screen',
            parameters=[
                param_file,
                {"device_type": device_type},
            ],
        ),
    ])
