#!/usr/bin/env python3
"""
GPS测试数据发布器
该脚本用于生成具有趋势性的GPS测试数据，按照预设范围发送
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import math
import time


class GPSTestPublisher(Node):
    def __init__(self):
        super().__init__('gps_test_publisher')
        
        # 发布GPS数据
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )
        
        # 参数配置
        self.declare_parameter('publish_frequency', 10.0)  # 发布频率
        self.declare_parameter('start_latitude', 39.9042)  # 起始纬度
        self.declare_parameter('start_longitude', 116.4074)  # 起始经度
        self.declare_parameter('latitude_range', 0.01)  # 纬度范围
        self.declare_parameter('longitude_range', 0.01)  # 经度范围
        self.declare_parameter('duration', 60.0)  # 持续时间
        self.declare_parameter('pattern', 'linear')  # 移动模式: linear, circular, random, no_fix
        
        # 获取参数
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.start_lat = self.get_parameter('start_latitude').value
        self.start_lon = self.get_parameter('start_longitude').value
        self.lat_range = self.get_parameter('latitude_range').value
        self.lon_range = self.get_parameter('longitude_range').value
        self.duration = self.get_parameter('duration').value
        self.pattern = self.get_parameter('pattern').value
        
        # 状态变量
        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_gps_data)
        
        self.get_logger().info('GPS测试数据发布器已启动')
        self.get_logger().info(f'配置: 频率={self.publish_frequency}Hz, 起始位置=({self.start_lat}, {self.start_lon})')
        self.get_logger().info(f'范围: 纬度={self.lat_range}, 经度={self.lon_range}, 持续时间={self.duration}秒')
        self.get_logger().info(f'移动模式: {self.pattern}')
    
    def get_gps_position(self, elapsed_time):
        """根据时间和模式计算GPS位置"""
        # 计算时间比例
        time_ratio = min(elapsed_time / self.duration, 1.0)
        
        if self.pattern == 'linear':
            # 线性移动模式
            lat = self.start_lat + self.lat_range * time_ratio
            lon = self.start_lon + self.lon_range * time_ratio
        
        elif self.pattern == 'circular':
            # 圆形移动模式
            angle = 2 * math.pi * time_ratio
            lat = self.start_lat + self.lat_range * math.sin(angle) / 2
            lon = self.start_lon + self.lon_range * math.cos(angle) / 2
        
        elif self.pattern == 'random':
            # 随机移动模式（带趋势）
            base_lat = self.start_lat + self.lat_range * time_ratio
            base_lon = self.start_lon + self.lon_range * time_ratio
            
            # 添加随机噪声
            noise_lat = (math.sin(elapsed_time * 2) * 0.001)
            noise_lon = (math.cos(elapsed_time * 1.5) * 0.001)
            
            lat = base_lat + noise_lat
            lon = base_lon + noise_lon
        
        else:
            # 默认线性模式
            lat = self.start_lat + self.lat_range * time_ratio
            lon = self.start_lon + self.lon_range * time_ratio
        
        return lat, lon
    
    def publish_gps_data(self):
        """发布GPS数据"""
        elapsed_time = time.time() - self.start_time
        
        # 检查是否超出持续时间
        if elapsed_time > self.duration:
            self.get_logger().info('GPS测试数据发布完成')
            self.timer.cancel()
            return
        
        # 创建GPS消息
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        
        if self.pattern == 'no_fix':
            # 生成无定位数据，与用户提供的格式匹配
            gps_msg.latitude = float('nan')
            gps_msg.longitude = float('nan')
            gps_msg.altitude = float('nan')
            
            # 设置状态为NO_FIX
            gps_msg.status.status = -1  # NO_FIX
            gps_msg.status.service = 1  # GPS
            
            # 设置协方差为全0
            gps_msg.position_covariance = [0.0] * 9
            gps_msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
            
            # 定期输出状态
            if int(elapsed_time) % 5 == 0 and elapsed_time - int(elapsed_time) < 0.1:
                self.get_logger().info(f'发布GPS数据: 状态=NO_FIX, 经纬度=NaN, 时间={elapsed_time:.1f}s')
        else:
            # 计算GPS位置
            lat, lon = self.get_gps_position(elapsed_time)
            
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = 10.0  # 固定高度
            
            # 设置状态和精度
            gps_msg.status.status = 0  # GPS_FIX
            gps_msg.status.service = 1  # GPS
            
            # 设置协方差（模拟中等精度）
            gps_msg.position_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.1
            ]
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            # 定期输出状态
            if int(elapsed_time) % 5 == 0 and elapsed_time - int(elapsed_time) < 0.1:
                self.get_logger().info(f'发布GPS数据: 纬度={lat:.6f}, 经度={lon:.6f}, 时间={elapsed_time:.1f}s')
        
        # 发布消息
        self.gps_pub.publish(gps_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = GPSTestPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
