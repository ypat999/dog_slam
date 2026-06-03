import os
from launch import LaunchDescription
from launch_ros.actions import Node


def get_gps_serial():
    # 根据实际设备优先级选择串口设备
    preferred_ports = ['/dev/RTK', '/dev/ttyGPS0', '/dev/ttyIMU2']
    for port in preferred_ports:
        if os.path.exists(port):
            return port
    return '/dev/RTK'  # 默认值


def generate_launch_description():
    gps_port = get_gps_serial()

    return LaunchDescription([
        Node(
            package='gps_driver',
            executable='gps_driver_node',
            name='gps_driver_node',
            output='screen',
            parameters=[{
                'gps_port': gps_port,
                'baudrate': 115200,
                'frame_id': 'gps_link'
            }]
        )
    ])
