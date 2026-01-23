#!/usr/bin/env python3
"""
GPS数据模拟器
在没有真实GPS设备时，模拟GPS数据用于测试GPS融合功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import math
import random
import time


class GPSSimulator(Node):
    def __init__(self):
        super().__init__('gps_simulator')
        
        # 参数
        self.declare_parameter('simulation_mode', 'fixed')  # fixed, random, trajectory
        self.declare_parameter('fixed_lat', 39.9042)        # 北京纬度
        self.declare_parameter('fixed_lon', 116.4074)       # 北京经度
        self.declare_parameter('fixed_alt', 50.0)           # 海拔50米
        self.declare_parameter('publish_rate', 1.0)         # 发布频率Hz
        self.declare_parameter('gps_quality', 0.5)          # GPS质量（0-1）
        self.declare_parameter('add_noise', True)           # 是否添加噪声
        
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.fixed_lat = self.get_parameter('fixed_lat').value
        self.fixed_lon = self.get_parameter('fixed_lon').value
        self.fixed_alt = self.get_parameter('fixed_alt').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.gps_quality = self.get_parameter('gps_quality').value
        self.add_noise = self.get_parameter('add_noise').value
        
        # GPS发布器
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )
        
        # 轨迹模拟变量
        self.current_lat = self.fixed_lat
        self.current_lon = self.fixed_lon
        self.trajectory_step = 0
        
        # 定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_gps_data)
        
        self.get_logger().info('GPS模拟器已启动')
        self.get_logger().info(f'模拟模式: {self.simulation_mode}')
        self.get_logger().info(f'发布频率: {self.publish_rate}Hz')
        self.get_logger().info(f'GPS质量: {self.gps_quality}')
    
    def generate_gps_noise(self, base_value, noise_level):
        """生成GPS噪声"""
        if not self.add_noise:
            return base_value
        
        # 根据GPS质量调整噪声水平
        effective_noise = noise_level * (1.0 - self.gps_quality)
        
        # 添加高斯噪声
        noise = random.gauss(0, effective_noise)
        return base_value + noise
    
    def simulate_trajectory(self):
        """模拟移动轨迹"""
        # 简单的圆形轨迹
        radius = 0.0001  # 约11米半径
        center_lat = self.fixed_lat
        center_lon = self.fixed_lon
        
        angle = self.trajectory_step * 0.1  # 每次增加0.1弧度
        
        # 计算新位置（简化计算，适用于小范围）
        delta_lat = radius * math.cos(angle)
        delta_lon = radius * math.sin(angle)
        
        self.current_lat = center_lat + delta_lat
        self.current_lon = center_lon + delta_lon
        
        self.trajectory_step += 1
    
    def publish_gps_data(self):
        """发布GPS数据"""
        gps_msg = NavSatFix()
        
        # 设置header
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        
        # 设置GPS状态
        gps_msg.status = NavSatStatus()
        
        # 根据GPS质量决定状态
        if random.random() < self.gps_quality:
            gps_msg.status.status = 0  # STATUS_FIX
            gps_msg.status.service = 1  # SERVICE_GPS
        else:
            gps_msg.status.status = -1  # STATUS_NO_FIX
            gps_msg.status.service = 1
        
        # 生成GPS数据
        if gps_msg.status.status == 0:  # 有定位
            if self.simulation_mode == 'fixed':
                lat = self.fixed_lat
                lon = self.fixed_lon
                alt = self.fixed_alt
            elif self.simulation_mode == 'random':
                # 在固定点周围随机移动
                lat = self.generate_gps_noise(self.fixed_lat, 0.0001)  # 约11米
                lon = self.generate_gps_noise(self.fixed_lon, 0.0001)
                alt = self.generate_gps_noise(self.fixed_alt, 5.0)     # 5米高度噪声
            elif self.simulation_mode == 'trajectory':
                self.simulate_trajectory()
                lat = self.generate_gps_noise(self.current_lat, 0.00005)  # 约5.5米
                lon = self.generate_gps_noise(self.current_lon, 0.00005)
                alt = self.generate_gps_noise(self.fixed_alt, 3.0)        # 3米高度噪声
            else:
                lat = self.fixed_lat
                lon = self.fixed_lon
                alt = self.fixed_alt
            
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt
            
            # 设置位置协方差（根据GPS质量）
            accuracy = 1.0 + (10.0 * (1.0 - self.gps_quality))  # 1-11米精度
            gps_msg.position_covariance = [
                accuracy**2, 0.0, 0.0,
                0.0, accuracy**2, 0.0,
                0.0, 0.0, (accuracy * 2)**2  # 高度精度较差
            ]
            gps_msg.position_covariance_type = 2  # DIAGONAL_KNOWN
            
            self.get_logger().debug(f'发布GPS数据: lat={lat:.6f}, lon={lon:.6f}, 精度={accuracy:.1f}m')
            
        else:  # 无定位
            gps_msg.latitude = float('nan')
            gps_msg.longitude = float('nan')
            gps_msg.altitude = float('nan')
            gps_msg.position_covariance = [float('nan')] * 9
            gps_msg.position_covariance_type = 0  # UNKNOWN
            
            self.get_logger().debug('发布无定位GPS数据')
        
        # 发布GPS数据
        self.gps_pub.publish(gps_msg)
        
        # 定期输出状态信息
        if self.get_clock().now().nanoseconds % 10_000_000_000 < 100_000:  # 每10秒输出一次
            status = "有定位" if gps_msg.status.status == 0 else "无定位"
            self.get_logger().info(f'GPS模拟状态: {status}, 质量={self.gps_quality}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        simulator = GPSSimulator()
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()