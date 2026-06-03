import os
from launch import LaunchDescription
from launch_ros.actions import Node


def get_gps_serial():
    # 根据实际设备优先级选择串口设备
    preferred_ports = ['/dev/gps_device', '/dev/ttyUSB0']
    for port in preferred_ports:
        if os.path.exists(port):
            return port
    return '/dev/ttyUSB0'  # 默认值


def generate_launch_description():
    gps_port = get_gps_serial()

    return LaunchDescription([
        Node(
            package='ntrip_nmea_driver',
            executable='ntrip_nmea_parser_node',
            name='ntrip_nmea_parser_node',
            output='screen',
            parameters=[{
                'gps_port': gps_port,
                'baudrate': 115200,
                'gps_ntrip_userpwd': 'gps_link',
                'gps_ntrip_server': '203.107.45.154',
                'ntrip_caster_port': 8003,
                'use_ntrip': False,
                'use_utc_time': False
            }]
        )
    ])
