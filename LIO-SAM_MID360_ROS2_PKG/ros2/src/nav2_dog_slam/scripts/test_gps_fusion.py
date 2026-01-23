#!/usr/bin/env python3
"""
GPS融合测试脚本
该脚本用于测试GPS与LIO-SAM的融合效果
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class GPSFusionTester(Node):
    def __init__(self):
        super().__init__('gps_fusion_tester')
        
        # 订阅GPS融合后的里程计
        self.gps_fused_sub = self.create_subscription(
            Odometry,
            '/odometry/gps_fused',
            self.gps_fused_callback,
            10
        )
        
        # 订阅原始LIO-SAM里程计
        self.lio_odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.lio_odom_callback,
            10
        )
        
        # 订阅AMCL定位结果
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # 测试数据存储
        self.gps_fused_data = []
        self.lio_odom_data = []
        self.amcl_pose_data = []
        
        # 定时器用于定期输出测试结果
        self.timer = self.create_timer(5.0, self.print_results)
        
        self.get_logger().info('GPS融合测试节点已启动')
    
    def gps_fused_callback(self, msg):
        """处理GPS融合后的里程计数据"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # 存储数据用于分析
        self.gps_fused_data.append({
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        # 限制数据量
        if len(self.gps_fused_data) > 100:
            self.gps_fused_data.pop(0)
    
    def lio_odom_callback(self, msg):
        """处理LIO-SAM原始里程计数据"""
        position = msg.pose.pose.position
        
        self.lio_odom_data.append({
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        if len(self.lio_odom_data) > 100:
            self.lio_odom_data.pop(0)
    
    def amcl_pose_callback(self, msg):
        """处理AMCL定位数据"""
        position = msg.pose.pose.position
        
        self.amcl_pose_data.append({
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'timestamp': self.get_clock().now().nanoseconds
        })
        
        if len(self.amcl_pose_data) > 100:
            self.amcl_pose_data.pop(0)
    
    def calculate_position_variance(self, data):
        """计算位置数据的方差"""
        if len(data) < 2:
            return 0.0, 0.0, 0.0
        
        x_values = [d['x'] for d in data]
        y_values = [d['y'] for d in data]
        
        x_mean = sum(x_values) / len(x_values)
        y_mean = sum(y_values) / len(y_values)
        
        x_variance = sum((x - x_mean) ** 2 for x in x_values) / len(x_values)
        y_variance = sum((y - y_mean) ** 2 for y in y_values) / len(y_values)
        
        return x_variance, y_variance, math.sqrt(x_variance + y_variance)
    
    def print_results(self):
        """定期输出测试结果"""
        if len(self.gps_fused_data) > 10:
            gps_x_var, gps_y_var, gps_total_var = self.calculate_position_variance(self.gps_fused_data)
            lio_x_var, lio_y_var, lio_total_var = self.calculate_position_variance(self.lio_odom_data)
            
            self.get_logger().info('=== GPS融合测试结果 ===')
            self.get_logger().info(f'GPS融合数据点数: {len(self.gps_fused_data)}')
            self.get_logger().info(f'GPS融合位置方差 - X: {gps_x_var:.6f}, Y: {gps_y_var:.6f}, 总: {gps_total_var:.6f}')
            self.get_logger().info(f'LIO-SAM位置方差 - X: {lio_x_var:.6f}, Y: {lio_y_var:.6f}, 总: {lio_total_var:.6f}')
            
            # 计算改进比例
            if lio_total_var > 0:
                improvement = (lio_total_var - gps_total_var) / lio_total_var * 100
                self.get_logger().info(f'定位稳定性改进: {improvement:.2f}%')
            
            self.get_logger().info('========================')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = GPSFusionTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()