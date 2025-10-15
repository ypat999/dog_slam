#!/usr/bin/env python3
"""
简单的IMU低通滤波器
直接集成到现有流程中，用于消除IMU数据的大幅跳变噪声
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque


class SimpleImuFilter(Node):
    def __init__(self):
        super().__init__('simple_imu_filter')
        
        # 参数设置
        self.window_size = 10  # 移动平均窗口大小
        self.jump_threshold = 2.0  # 跳变检测阈值（标准差的倍数）
        
        # 初始化缓冲区
        self.angular_velocity_buffers = {
            'x': deque(maxlen=self.window_size),
            'y': deque(maxlen=self.window_size),
            'z': deque(maxlen=self.window_size)
        }
        self.linear_acceleration_buffers = {
            'x': deque(maxlen=self.window_size),
            'y': deque(maxlen=self.window_size),
            'z': deque(maxlen=self.window_size)
        }
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10)
        
        self.publisher = self.create_publisher(Imu, '/livox/imu_filtered', 10)
        
        self.get_logger().info('简单IMU滤波器已启动')
        self.get_logger().info(f'窗口大小: {self.window_size}')
        self.get_logger().info(f'跳变阈值: {self.jump_threshold}')

    def detect_and_handle_jump(self, new_value, buffer, axis_name, data_type):
        """检测并处理跳变值"""
        if len(buffer) < 3:  # 缓冲区数据不足，直接返回原值
            return new_value
            
        # 计算历史数据的均值和标准差
        historical_data = list(buffer)
        mean = np.mean(historical_data)
        std = np.std(historical_data)
        
        # 检测跳变
        if std > 0:  # 避免除零
            jump_ratio = abs(new_value - mean) / std
            if jump_ratio > self.jump_threshold:
                self.get_logger().debug(f'检测到{data_type} {axis_name}轴跳变: {new_value:.3f} -> {mean:.3f} (ratio: {jump_ratio:.2f})')
                return mean  # 用历史均值替代跳变值
        
        return new_value

    def moving_average_filter(self, new_value, buffer):
        """移动平均滤波"""
        buffer.append(new_value)
        if len(buffer) > 0:
            return np.mean(list(buffer))
        return new_value

    def imu_callback(self, msg):
        """IMU数据回调函数"""
        # 创建滤波后的消息
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'livox_frame'
        
        # 处理角速度
        angular_velocity = {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z
        }
        
        # 处理线加速度
        linear_acceleration = {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        }
        
        # 对每个轴进行滤波处理
        for axis in ['x', 'y', 'z']:
            # 角速度滤波
            filtered_ang_vel = angular_velocity[axis]
            filtered_ang_vel = self.detect_and_handle_jump(
                filtered_ang_vel, 
                self.angular_velocity_buffers[axis], 
                axis, 
                '角速度'
            )
            filtered_ang_vel = self.moving_average_filter(
                filtered_ang_vel, 
                self.angular_velocity_buffers[axis]
            )
            
            # 线加速度滤波
            filtered_lin_acc = linear_acceleration[axis]
            filtered_lin_acc = self.detect_and_handle_jump(
                filtered_lin_acc, 
                self.linear_acceleration_buffers[axis], 
                axis, 
                '线加速度'
            )
            filtered_lin_acc = self.moving_average_filter(
                filtered_lin_acc, 
                self.linear_acceleration_buffers[axis]
            )
            
            # 设置滤波后的值
            setattr(filtered_msg.angular_velocity, axis, filtered_ang_vel)
            setattr(filtered_msg.linear_acceleration, axis, filtered_lin_acc)
        
        # 复制协方差矩阵
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        
        # 发布滤波后的数据
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_filter = SimpleImuFilter()
        rclpy.spin(imu_filter)
    except KeyboardInterrupt:
        pass
    finally:
        if 'imu_filter' in locals():
            imu_filter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()