from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([

        # 启动 arc_lvio 节点
        Node(
            package='arc_lvio',          # 包名
            executable='arc_lvio',       # 可执行文件名（就是 build 出的 arc_lvio）
            name='arc_lvio_node',        # ROS 节点名（可改）
            output='screen',             # 日志输出到终端
        ),
    ])
