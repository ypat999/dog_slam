#!/usr/bin/env python3
"""
GPS数据预处理节点
处理GPS无定位状态和NaN值，确保只有有效的GPS数据进入EKF滤波器
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import math


class GPSPreprocessor(Node):
    def __init__(self):
        super().__init__('gps_preprocessor')
        
        # 参数
        self.declare_parameter('min_satellites', 4)  # 最小卫星数
        self.declare_parameter('max_hdop', 2.0)      # 最大水平精度因子
        self.declare_parameter('min_accuracy', 0.1)  # 最小精度（米）
        self.declare_parameter('status_threshold', 0)  # 状态阈值（0=FIX, -1=NO_FIX）
        
        self.min_satellites = self.get_parameter('min_satellites').value
        self.max_hdop = self.get_parameter('max_hdop').value
        self.min_accuracy = self.get_parameter('min_accuracy').value
        self.status_threshold = self.get_parameter('status_threshold').value
        
        # 订阅原始GPS数据
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # 发布处理后的GPS数据
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix_filtered',
            10
        )
        
        # 发布GPS可用状态
        self.gps_status_pub = self.create_publisher(
            Bool,
            '/gps/status',
            10
        )
        
        # 状态变量
        self.last_valid_gps = None
        self.gps_available = False
        self.gps_quality_counter = 0
        
        self.get_logger().info('GPS预处理节点已启动')
        self.get_logger().info(f'参数配置: 最小卫星数={self.min_satellites}, 最大HDOP={self.max_hdop}')
    
    def is_valid_gps_data(self, gps_msg):
        """检查GPS数据是否有效"""
        
        # 检查状态
        if gps_msg.status.status < self.status_threshold:
            self.get_logger().debug(f'GPS状态无效: {gps_msg.status.status}')
            return False
        
        # 检查经纬度是否为NaN
        if math.isnan(gps_msg.latitude) or math.isnan(gps_msg.longitude):
            self.get_logger().debug('GPS经纬度包含NaN值')
            return False
        
        # 检查经纬度范围
        if not (-90 <= gps_msg.latitude <= 90) or not (-180 <= gps_msg.longitude <= 180):
            self.get_logger().debug(f'GPS经纬度超出范围: lat={gps_msg.latitude}, lon={gps_msg.longitude}')
            return False
        
        # 检查位置协方差（如果有）
        if gps_msg.position_covariance_type > 0:
            # 计算水平精度（假设对角线协方差）
            h_accuracy = math.sqrt(gps_msg.position_covariance[0] + gps_msg.position_covariance[4])
            if h_accuracy > self.min_accuracy:
                self.get_logger().debug(f'GPS精度过低: {h_accuracy:.2f}m')
                return False
        
        return True
    
    def calculate_hdop(self, gps_msg):
        """计算水平精度因子（HDOP）"""
        if gps_msg.position_covariance_type > 0:
            # 从协方差矩阵计算HDOP
            h_accuracy = math.sqrt(gps_msg.position_covariance[0] + gps_msg.position_covariance[4])
            # 简化计算：假设1米精度对应HDOP=1
            return h_accuracy
        return float('inf')
    
    def gps_callback(self, msg):
        """处理原始GPS数据"""
        
        # 检查数据有效性
        is_valid = self.is_valid_gps_data(msg)
        
        if is_valid:
            # 计算HDOP
            hdop = self.calculate_hdop(msg)
            
            # 更新质量计数器
            self.gps_quality_counter += 1
            if self.gps_quality_counter > 5:  # 连续5个有效数据点
                self.gps_available = True
            
            # 存储最后一个有效GPS数据
            self.last_valid_gps = msg
            
            # 发布处理后的数据
            filtered_msg = NavSatFix()
            filtered_msg.header = msg.header
            filtered_msg.status = msg.status
            filtered_msg.latitude = msg.latitude
            filtered_msg.longitude = msg.longitude
            filtered_msg.altitude = msg.altitude
            filtered_msg.position_covariance = msg.position_covariance
            filtered_msg.position_covariance_type = msg.position_covariance_type
            
            # 添加质量信息到header中
            filtered_msg.header.frame_id = f"gps_hdop_{hdop:.1f}"
            
            self.gps_pub.publish(filtered_msg)
            
            self.get_logger().debug(f'发布有效GPS数据: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, HDOP={hdop:.1f}')
            
        else:
            # GPS数据无效
            self.gps_quality_counter = max(0, self.gps_quality_counter - 1)
            if self.gps_quality_counter == 0:
                self.gps_available = False
            
            self.get_logger().debug('GPS数据无效，已过滤')
        
        # 发布GPS状态
        status_msg = Bool()
        status_msg.data = self.gps_available
        self.gps_status_pub.publish(status_msg)
        
        # 定期输出状态信息
        if self.get_clock().now().nanoseconds % 10_000_000_000 < 100_000:  # 每10秒输出一次
            self.get_logger().info(f'GPS状态: 可用={self.gps_available}, 质量计数器={self.gps_quality_counter}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        preprocessor = GPSPreprocessor()
        rclpy.spin(preprocessor)
    except KeyboardInterrupt:
        pass
    finally:
        preprocessor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()