import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 获取参数配置文件路径
    parameter_config = PathJoinSubstitution([
        get_package_share_directory('localization'),
        'config',
        'ros_params_loc.yaml'
    ])
    
    # 定义 launch 参数，类型为 'mapping' 或 'localization'
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='mapping',
        description='Choose between "mapping" or "localization" mode'
    )

    # 启动 Node，使用 LaunchConfiguration 来传递模式类型
    localization = Node(
        package='localization',
        executable='localization',
        parameters=[
            parameter_config,
            {
                "rtk_localization_mode": LaunchConfiguration('type')  # 使用字典来传递 LaunchConfiguration
            }
        ],
        remappings=[],
        output='screen'
    )

    return LaunchDescription([
        type_arg,
        localization
    ])

