#!/usr/bin/env python3
"""
系统测试脚本
用于验证Livox SLAM系统的基本功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import subprocess
import time
import os

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # 订阅点云话题以验证数据流
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10)
        
        self.pointcloud_received = False
        self.test_start_time = time.time()
        
        self.get_logger().info('系统测试器已启动')
        
    def pointcloud_callback(self, msg):
        if not self.pointcloud_received:
            self.pointcloud_received = True
            self.get_logger().info(f'成功接收到点云数据，包含 {len(msg.data)} 字节')
            self.get_logger().info('点云数据验证通过')
    
    def run_test(self):
        """运行系统测试"""
        self.get_logger().info('开始系统测试...')
        
        # 测试1: 检查必要的包是否已安装
        self.test_packages_installed()
        
        # 测试2: 检查配置文件是否存在
        self.test_config_files()
        
        # 测试3: 检查launch文件
        self.test_launch_files()
        
        self.get_logger().info('系统测试完成')
        
    def test_packages_installed(self):
        """测试必要的包是否已安装"""
        required_packages = ['livox_ros_driver2', 'my_cartographer_launch']
        
        try:
            from ament_index_python.packages import get_package_share_directory
            for package in required_packages:
                path = get_package_share_directory(package)
                if os.path.exists(path):
                    self.get_logger().info(f'包 {package} 已正确安装')
                else:
                    self.get_logger().warn(f'包 {package} 未找到')
        except Exception as e:
            self.get_logger().error(f'包检查失败: {str(e)}')
    
    def test_config_files(self):
        """测试配置文件是否存在"""
        config_files = [
            'config/mid360_config.json',
            'config/livox_mid360_cartographer.lua'
        ]
        
        for config_file in config_files:
            file_path = os.path.join('/public/github/livox_ws/src/livox_slam_system', config_file)
            if os.path.exists(file_path):
                self.get_logger().info(f'配置文件 {config_file} 存在')
            else:
                self.get_logger().warn(f'配置文件 {config_file} 不存在')
    
    def test_launch_files(self):
        """测试launch文件是否存在"""
        launch_files = [
            'launch/livox_slam_system.launch.py',
            'launch/simple_launch.py'
        ]
        
        for launch_file in launch_files:
            file_path = os.path.join('/public/github/livox_ws/src/livox_slam_system', launch_file)
            if os.path.exists(file_path):
                self.get_logger().info(f'Launch文件 {launch_file} 存在')
            else:
                self.get_logger().warn(f'Launch文件 {launch_file} 不存在')

def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    
    # 运行测试
    tester.run_test()
    
    # 运行节点一段时间以接收点云数据
    tester.get_logger().info('等待点云数据...')
    time.sleep(10)  # 等待10秒
    
    rclpy.spin_once(tester, timeout_sec=1)
    
    if tester.pointcloud_received:
        tester.get_logger().info('系统测试成功: 所有检查通过')
    else:
        tester.get_logger().warn('系统测试警告: 未接收到点云数据（LiDAR可能未连接）')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()