from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # 启动 arc_mapping 节点
        Node(
            package='arc_mapping',          # 包名
            executable='arc_mapping',       # 可执行文件名（就是 build 出的 arc_mapping）
            name='arc_mapping_node',        # ROS 节点名（可改）
            output='screen',             # 日志输出到终端
        ),
    ])
