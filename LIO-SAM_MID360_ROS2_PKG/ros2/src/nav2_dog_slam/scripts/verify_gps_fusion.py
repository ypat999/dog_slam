#!/usr/bin/env python3
"""
GPS融合验证脚本
用于验证建图后GPS融合的情况
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math


class GPSFusionVerifier(Node):
    def __init__(self):
        super().__init__('gps_fusion_verifier')
        
        # 订阅GPS数据
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # 订阅过滤后的GPS数据
        self.gps_filtered_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix_filtered',
            self.gps_filtered_callback,
            10
        )
        
        # 订阅GPS里程计
        self.gps_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/gps',
            self.gps_odom_callback,
            10
        )
        
        # 订阅GPS融合后的里程计
        self.gps_fused_sub = self.create_subscription(
            Odometry,
            '/odometry/gps_fused',
            self.gps_fused_callback,
            10
        )
        
        # 订阅LIO-SAM里程计
        self.lio_odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.lio_odom_callback,
            10
        )
        
        # 数据存储
        self.gps_data = []
        self.gps_filtered_data = []
        self.gps_odom_data = []
        self.gps_fused_data = []
        self.lio_odom_data = []
        
        # 统计信息
        self.gps_count = 0
        self.gps_filtered_count = 0
        self.gps_odom_count = 0
        self.gps_fused_count = 0
        self.lio_odom_count = 0
        
        # 定时器用于定期输出验证结果
        self.timer = self.create_timer(5.0, self.print_verification_results)
        
        self.get_logger().info('GPS融合验证节点已启动')
        self.get_logger().info('开始收集GPS融合相关数据...')
    
    def gps_callback(self, msg):
        """处理原始GPS数据"""
        self.gps_count += 1
        self.gps_data.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'status': msg.status.status
        })
        
        # 限制数据量
        if len(self.gps_data) > 100:
            self.gps_data.pop(0)
    
    def gps_filtered_callback(self, msg):
        """处理过滤后的GPS数据"""
        self.gps_filtered_count += 1
        self.gps_filtered_data.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'status': msg.status.status
        })
        
        if len(self.gps_filtered_data) > 100:
            self.gps_filtered_data.pop(0)
    
    def gps_odom_callback(self, msg):
        """处理GPS里程计数据"""
        self.gps_odom_count += 1
        self.gps_odom_data.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        })
        
        if len(self.gps_odom_data) > 100:
            self.gps_odom_data.pop(0)
    
    def gps_fused_callback(self, msg):
        """处理GPS融合后的里程计数据"""
        self.gps_fused_count += 1
        self.gps_fused_data.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        })
        
        if len(self.gps_fused_data) > 100:
            self.gps_fused_data.pop(0)
    
    def lio_odom_callback(self, msg):
        """处理LIO-SAM里程计数据"""
        self.lio_odom_count += 1
        self.lio_odom_data.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        })
        
        if len(self.lio_odom_data) > 100:
            self.lio_odom_data.pop(0)
    
    def calculate_position_change(self, data):
        """计算位置变化"""
        if len(data) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(data)):
            dx = data[i]['x'] - data[i-1]['x']
            dy = data[i]['y'] - data[i-1]['y']
            distance = math.sqrt(dx*dx + dy*dy)
            total_distance += distance
        
        return total_distance
    
    def print_verification_results(self):
        """打印验证结果"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('GPS融合验证结果')
        self.get_logger().info('='*60)
        
        # 数据接收统计
        self.get_logger().info(f'原始GPS数据: {self.gps_count} 条')
        self.get_logger().info(f'过滤后GPS数据: {self.gps_filtered_count} 条')
        self.get_logger().info(f'GPS里程计数据: {self.gps_odom_count} 条')
        self.get_logger().info(f'GPS融合数据: {self.gps_fused_count} 条')
        self.get_logger().info(f'LIO-SAM里程计数据: {self.lio_odom_count} 条')
        
        # 数据质量分析
        if self.gps_data:
            valid_gps = sum(1 for d in self.gps_data if not math.isnan(d['latitude']) and d['status'] >= 0)
            invalid_gps = len(self.gps_data) - valid_gps
            self.get_logger().info(f'GPS数据质量: 有效={valid_gps}, 无效={invalid_gps}')
        
        # 位置变化分析
        if self.gps_fused_data and self.lio_odom_data:
            gps_fused_distance = self.calculate_position_change(self.gps_fused_data)
            lio_distance = self.calculate_position_change(self.lio_odom_data)
            
            self.get_logger().info(f'位置变化分析:')
            self.get_logger().info(f'  GPS融合里程: {gps_fused_distance:.2f} 米')
            self.get_logger().info(f'  LIO-SAM里程: {lio_distance:.2f} 米')
            
            if lio_distance > 0:
                difference = abs(gps_fused_distance - lio_distance) / lio_distance * 100
                self.get_logger().info(f'  差异: {difference:.2f}%')
        
        # 系统状态评估
        if self.gps_fused_count > 0:
            self.get_logger().info('✅ GPS融合系统正常工作')
        else:
            self.get_logger().info('❌ GPS融合系统未接收到数据')
        
        # 数据流程检查
        if self.gps_count > 0 and self.gps_filtered_count == 0:
            self.get_logger().info('⚠️  原始GPS数据已接收，但未经过滤')
        elif self.gps_filtered_count > 0 and self.gps_odom_count == 0:
            self.get_logger().info('⚠️  过滤后GPS数据已接收，但未转换为里程计')
        elif self.gps_odom_count > 0 and self.gps_fused_count == 0:
            self.get_logger().info('⚠️  GPS里程计数据已接收，但未融合')
        
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        verifier = GPSFusionVerifier()
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
