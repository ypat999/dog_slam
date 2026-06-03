#!/usr/bin/env python3
"""
odom_corrector.py - Odom修正节点

功能：
1. 接收 /initialpose 消息后，清空队列
2. 接收并记录200组 map->odom 的tf（带namespace支持）
3. 实时计算平均值
4. 如果接收到的tf与平均值偏差大于阈值（可配置，默认3米），则按照平均值自行发布一个/initialpose，并sleep 5秒
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
import math
import time
from collections import deque


class OdomCorrector(Node):
    def __init__(self):
        super().__init__('odom_corrector')

        # 声明参数
        self.declare_parameter('deviation_threshold', 3.0)  # 偏差阈值，单位：米
        self.declare_parameter('sample_count', 200)  # 采样数量
        self.declare_parameter('sleep_duration', 5.0)  # 修正后的休眠时间
        self.declare_parameter('tf_timeout', 1.0)  # TF查询超时时间
        self.declare_parameter('source_frame', 'map')  # 源坐标系
        self.declare_parameter('target_frame', 'odom')  # 目标坐标系
        self.declare_parameter('namespace', '')  # namespace，如果为空则自动获取

        # 获取参数
        self.deviation_threshold = self.get_parameter('deviation_threshold').value
        self.sample_count = self.get_parameter('sample_count').value
        self.sleep_duration = self.get_parameter('sleep_duration').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        namespace = self.get_parameter('namespace').value

        # 处理namespace
        if namespace:
            self.namespace = namespace
        else:
            # 自动从节点namespace获取
            self.namespace = self.get_namespace().rstrip('/')

        # 构建带namespace的frame_id
        if self.namespace and not self.namespace.startswith('/'):
            self.namespaced_source = f"/{self.namespace}/{self.source_frame}"
            self.namespaced_target = f"/{self.namespace}/{self.target_frame}"
        elif self.namespace:
            self.namespaced_source = f"{self.namespace}/{self.source_frame}"
            self.namespaced_target = f"{self.namespace}/{self.target_frame}"
        else:
            self.namespaced_source = self.source_frame
            self.namespaced_target = self.target_frame

        self.get_logger().info(f"Source frame: {self.namespaced_source}")
        self.get_logger().info(f"Target frame: {self.namespaced_target}")
        self.get_logger().info(f"Deviation threshold: {self.deviation_threshold}m")
        self.get_logger().info(f"Sample count: {self.sample_count}")

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅 /initialpose
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # 发布 /initialpose
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 数据队列
        self.tf_samples = deque(maxlen=self.sample_count)
        self.collecting = False
        self.sleeping = False

        # 定时器用于采集TF
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.get_logger().info("OdomCorrector initialized and ready")

    def initialpose_callback(self, msg):
        """接收到 /initialpose 后，清空队列，开始新的采集"""
        if self.sleeping:
            self.get_logger().warn("Currently in sleep mode, ignoring initialpose")
            return

        self.get_logger().info("Received /initialpose, clearing queue and starting new collection")
        self.tf_samples.clear()
        self.collecting = True

    def timer_callback(self):
        """定时采集TF数据"""
        if not self.collecting or self.sleeping:
            return

        try:
            # 查询 TF
            transform = self.tf_buffer.lookup_transform(
                self.namespaced_source,
                self.namespaced_target,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )

            # 提取位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # 提取四元数
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # 添加到队列
            self.tf_samples.append({
                'x': x,
                'y': y,
                'z': z,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw
            })

            current_count = len(self.tf_samples)

            # 当采集到足够样本后，开始检测
            if current_count >= self.sample_count:
                self.check_and_correct(x, y, z, qx, qy, qz, qw)

        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {e}")

    def check_and_correct(self, current_x, current_y, current_z, qx, qy, qz, qw):
        """检查当前值与平均值的偏差，必要时进行修正"""
        # 计算平均值
        avg_x = sum(s['x'] for s in self.tf_samples) / len(self.tf_samples)
        avg_y = sum(s['y'] for s in self.tf_samples) / len(self.tf_samples)
        avg_z = sum(s['z'] for s in self.tf_samples) / len(self.tf_samples)

        # 计算四元数平均值（简化处理：使用最后一个样本的方向）
        # 更精确的做法是使用球面插值，但这里简化为使用历史平均值
        avg_qx = sum(s['qx'] for s in self.tf_samples) / len(self.tf_samples)
        avg_qy = sum(s['qy'] for s in self.tf_samples) / len(self.tf_samples)
        avg_qz = sum(s['qz'] for s in self.tf_samples) / len(self.tf_samples)
        avg_qw = sum(s['qw'] for s in self.tf_samples) / len(self.tf_samples)

        # 归一化四元数
        norm = math.sqrt(avg_qx**2 + avg_qy**2 + avg_qz**2 + avg_qw**2)
        if norm > 0:
            avg_qx /= norm
            avg_qy /= norm
            avg_qz /= norm
            avg_qw /= norm

        # 计算与平均值的距离
        distance = math.sqrt(
            (current_x - avg_x)**2 +
            (current_y - avg_y)**2 +
            (current_z - avg_z)**2
        )

        self.get_logger().info(
            f"Current: ({current_x:.3f}, {current_y:.3f}, {current_z:.3f}), "
            f"Avg: ({avg_x:.3f}, {avg_y:.3f}, {avg_z:.3f}), "
            f"Distance: {distance:.3f}m"
        )

        # 如果偏差超过阈值，发布修正的initialpose
        if distance > self.deviation_threshold:
            self.get_logger().warn(
                f"Deviation {distance:.3f}m exceeds threshold {self.deviation_threshold}m, "
                f"publishing correction"
            )
            self.publish_correction(avg_x, avg_y, avg_z, avg_qx, avg_qy, avg_qz, avg_qw)

    def publish_correction(self, x, y, z, qx, qy, qz, qw):
        """发布修正的initialpose"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.source_frame  # 使用不带namespace的frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position = Point(x=x, y=y, z=z)
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # 协方差矩阵 - 使用较小的值表示较高的置信度
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[14] = 0.25  # z
        cov[35] = 0.06853892326654787  # yaw
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published correction to /initialpose: "
            f"position=({x:.3f}, {y:.3f}, {z:.3f})"
        )

        # 清空队列并进入休眠
        self.tf_samples.clear()
        self.sleeping = True

        # 创建一次性定时器来恢复
        self.create_timer(self.sleep_duration, self.wake_up, callback_group=None)

    def wake_up(self):
        """从休眠中恢复"""
        self.get_logger().info("Waking up from sleep, resuming collection")
        self.sleeping = False
        self.collecting = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
