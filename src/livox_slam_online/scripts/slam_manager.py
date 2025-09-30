#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import subprocess
import os

class SLAMManager(Node):
    def __init__(self):
        super().__init__('slam_manager')
        
        # 创建订阅者接收点云数据
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # 初始化状态
        self.slam_process = None
        self.recording_process = None
        
        self.get_logger().info('SLAM Manager initialized')
        
    def pointcloud_callback(self, msg):
        self.get_logger().debug('Received point cloud data')
        
    def start_slam(self):
        """启动Cartographer SLAM进程"""
        try:
            # 这里应该启动Cartographer进程
            # 由于这是一个示例，我们只记录日志
            self.get_logger().info('Starting Cartographer SLAM process')
        except Exception as e:
            self.get_logger().error(f'Failed to start SLAM: {str(e)}')
            
    def start_recording(self):
        """启动数据录制进程"""
        try:
            # 启动rosbag录制进程
            self.recording_process = subprocess.Popen([
                'ros2', 'bag', 'record', '-o', '/tmp/livox_slam_recording',
                '/livox/lidar', '/tf', '/tf_static'
            ])
            self.get_logger().info('Started data recording')
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {str(e)}')
            
    def stop_slam(self):
        """停止SLAM进程"""
        if self.slam_process:
            self.slam_process.terminate()
            self.slam_process = None
            self.get_logger().info('Stopped SLAM process')
            
    def stop_recording(self):
        """停止数据录制"""
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process = None
            self.get_logger().info('Stopped data recording')

def main(args=None):
    rclpy.init(args=args)
    slam_manager = SLAMManager()
    
    # 启动SLAM和录制
    slam_manager.start_slam()
    slam_manager.start_recording()
    
    rclpy.spin(slam_manager)
    
    # 清理
    slam_manager.stop_recording()
    slam_manager.stop_slam()
    
    slam_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()