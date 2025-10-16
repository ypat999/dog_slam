#!/usr/bin/env python3
"""
增强型IMU低通滤波器 + 缓冲中继
1. 消除IMU数据大幅跳变噪声
2. 内部缓冲，保证Cartographer订阅BEST_EFFORT时不会丢包
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time


class BufferedImuFilter(Node):
    def __init__(self):
        super().__init__('buffered_imu_filter')
        
        # 参数设置
        self.window_size = 10  # 移动平均窗口大小
        self.jump_threshold = 2.0  # 跳变检测阈值（标准差的倍数）
        self.buffer_size = 50  # 内部缓冲长度，防止丢包
        
        # 统计变量
        self.stats_interval = 10.0  # 统计间隔（秒）
        self.raw_count = 0
        self.filtered_count = 0
        self.last_stats_time = time.time()
        self.raw_timestamps = deque()
        self.filtered_timestamps = deque()
        
        # 初始化缓冲区
        self.angular_velocity_buffers = {axis: deque(maxlen=self.window_size) for axis in ['x', 'y', 'z']}
        self.linear_acceleration_buffers = {axis: deque(maxlen=self.window_size) for axis in ['x', 'y', 'z']}
        self.relay_buffer = deque(maxlen=self.buffer_size)
        
        # QoS设置
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5000
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5000
        )
        
        # 创建订阅 /livox/imu
        self.subscription = self.create_subscription(Imu, '/livox/imu', self.imu_callback, qos_sub)
        # 发布 /livox/imu_filtered
        self.publisher = self.create_publisher(Imu, '/livox/imu_filtered', qos_pub)
        
        # 定时器：从缓冲区以固定频率发布
        self.timer = self.create_timer(0.001, self.publish_from_buffer)  # 1 kHz 尝试
        
        # 统计定时器：每10秒输出统计信息
        self.stats_timer = self.create_timer(self.stats_interval, self.publish_statistics)
        
        self.get_logger().info('增强型IMU滤波器已启动')

    def detect_and_handle_jump(self, new_value, buffer, axis_name, data_type):
        if len(buffer) < 3:
            return new_value
        historical_data = list(buffer)
        mean = np.mean(historical_data)
        std = np.std(historical_data)
        if std > 0:
            jump_ratio = abs(new_value - mean) / std
            if jump_ratio > self.jump_threshold:
                self.get_logger().debug(f'检测到{data_type} {axis_name}轴跳变: {new_value:.3f} -> {mean:.3f} (ratio: {jump_ratio:.2f})')
                return mean
        return new_value

    def moving_average_filter(self, new_value, buffer):
        buffer.append(new_value)
        return np.mean(list(buffer)) if len(buffer) > 0 else new_value

    def imu_callback(self, msg: Imu):
        """IMU数据回调 + 滤波 + 放入缓冲"""
        # 统计原始数据
        self.raw_count += 1
        current_time = time.time()
        self.raw_timestamps.append(current_time)
        
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'livox_frame'

        # 角速度与线加速度滤波
        for axis in ['x', 'y', 'z']:
            ang_val = self.detect_and_handle_jump(msg.angular_velocity.__getattribute__(axis),
                                                  self.angular_velocity_buffers[axis], axis, '角速度')
            ang_val = self.moving_average_filter(ang_val, self.angular_velocity_buffers[axis])
            setattr(filtered_msg.angular_velocity, axis, ang_val)

            lin_val = self.detect_and_handle_jump(msg.linear_acceleration.__getattribute__(axis),
                                                  self.linear_acceleration_buffers[axis], axis, '线加速度')
            lin_val = self.moving_average_filter(lin_val, self.linear_acceleration_buffers[axis])
            setattr(filtered_msg.linear_acceleration, axis, lin_val)

        # 复制协方差和方向
        filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        filtered_msg.orientation_covariance = msg.orientation_covariance
        filtered_msg.orientation = msg.orientation

        # 放入发布缓冲
        self.relay_buffer.append(filtered_msg)
        
        # 统计滤波后数据
        self.filtered_count += 1
        self.filtered_timestamps.append(time.time())

    def publish_from_buffer(self):
        """从缓冲区发布数据"""
        if len(self.relay_buffer) == 0:
            return
        msg = self.relay_buffer.popleft()
        self.publisher.publish(msg)

    def cleanup_old_timestamps(self):
        """清理过期的时间戳数据"""
        current_time = time.time()
        cutoff_time = current_time - 30.0  # 保留30秒的数据
        
        # 清理原始数据时间戳
        while self.raw_timestamps and self.raw_timestamps[0] < cutoff_time:
            self.raw_timestamps.popleft()
            
        # 清理滤波数据时间戳
        while self.filtered_timestamps and self.filtered_timestamps[0] < cutoff_time:
            self.filtered_timestamps.popleft()

    def calculate_frequency(self, timestamps):
        """计算频率"""
        if len(timestamps) < 2:
            return 0.0
            
        # 使用最近10秒的数据
        current_time = time.time()
        recent_timestamps = [t for t in timestamps if current_time - t <= 10.0]
        
        if len(recent_timestamps) < 2:
            return 0.0
            
        time_span = max(recent_timestamps) - min(recent_timestamps)
        if time_span > 0:
            return len(recent_timestamps) / time_span
        return 0.0

    def publish_statistics(self):
        """发布统计信息"""
        current_time = time.time()
        
        # 清理过期数据
        self.cleanup_old_timestamps()
        
        # 计算频率
        raw_freq = self.calculate_frequency(self.raw_timestamps)
        filtered_freq = self.calculate_frequency(self.filtered_timestamps)
        
        # 输出统计信息
        self.get_logger().info('=== IMU数据统计 (过去10秒) ===')
        self.get_logger().info(f'原始IMU数据:')
        self.get_logger().info(f'  - 消息计数: {self.raw_count}')
        self.get_logger().info(f'  - 频率: {raw_freq:.2f} Hz')
        
        self.get_logger().info(f'滤波后IMU数据:')
        self.get_logger().info(f'  - 消息计数: {self.filtered_count}')
        self.get_logger().info(f'  - 频率: {filtered_freq:.2f} Hz')
        
        # 数据质量评估
        expected_freq = 200.0  # Livox IMU预期频率
        if raw_freq > 0:
            raw_quality = min(100.0, (raw_freq / expected_freq) * 100)
            self.get_logger().info(f'  - 原始数据质量: {raw_quality:.1f}% (预期: {expected_freq}Hz)')
        
        if filtered_freq > 0 and raw_freq > 0:
            drop_rate = max(0.0, ((raw_freq - filtered_freq) / raw_freq) * 100)
            self.get_logger().info(f'  - 滤波丢包率: {drop_rate:.1f}%')
        
        self.get_logger().info(f'缓冲区状态:')
        self.get_logger().info(f'  - 原始时间戳数: {len(self.raw_timestamps)}')
        self.get_logger().info(f'  - 滤波时间戳数: {len(self.filtered_timestamps)}')
        self.get_logger().info(f'  - 中继缓冲区: {len(self.relay_buffer)}')
        self.get_logger().info('=' * 45)
        
        # 重置计数器
        self.raw_count = 0
        self.filtered_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = BufferedImuFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
