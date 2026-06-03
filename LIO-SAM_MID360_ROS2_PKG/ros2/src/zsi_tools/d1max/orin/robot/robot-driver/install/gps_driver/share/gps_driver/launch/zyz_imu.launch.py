'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-04-26 15:54:07
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-04-28 15:19:15
FilePath: /src/ros2_example/launch/lpms_be1_launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def get_gnss_serial():
    gnss_serial = '/dev/ttyUSB0'
    if os.path.exists('/dev/ttyIMU2'):
        gnss_serial = '/dev/ttyIMU2'
    
    return gnss_serial

def generate_launch_description():

    imu_port = get_gnss_serial()
    return LaunchDescription([
        Node(
            package = 'zyz_imu',
            executable = 'zyz_imu_node',
            output = 'screen',
            parameters=[{
                "imu_port": imu_port,
                "topic": "imu",
                "imu_baudrate": 115200,
                # 是否使用自研积分算法，True为使用自研的积分，Flase为使用imu自带的积分，默认为False
            }]
        )
    ])
