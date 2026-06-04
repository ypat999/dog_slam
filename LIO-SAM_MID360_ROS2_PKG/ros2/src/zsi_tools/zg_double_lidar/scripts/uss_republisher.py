#!/usr/bin/env python3
"""
uss_republisher.py - USS Range消息修复节点

功能：
1. 订阅原始USS Range消息
2. 填充min_range、max_range、radiation_type和field_of_view字段
3. 发布到/rkbot命名空间
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UssRepublisher(Node):
    def __init__(self):
        super().__init__('uss_republisher')

        # 声明参数
        self.declare_parameter('min_range', 0.02)      # 最小检测距离 (m)
        self.declare_parameter('max_range', 2.0)       # 最大检测距离 (m)
        self.declare_parameter('field_of_view', 0.2)   # 视野角度 (弧度)
        self.declare_parameter('radiation_type', 0)    # 辐射类型: 0=ultrasound, 1=infrared
        self.declare_parameter('namespace', '/rkbot')   # 目标命名空间

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.radiation_type = self.get_parameter('radiation_type').value
        self.namespace = self.get_parameter('namespace').value.rstrip('/')

        # 订阅原始USS话题
        self.sub_left = self.create_subscription(
            Range,
            '/uss_driver/uss_left/range',
            self.callback_left,
            10
        )
        self.sub_right = self.create_subscription(
            Range,
            '/uss_driver/uss_right/range',
            self.callback_right,
            10
        )

        # 发布修复后的Range消息
        self.pub_left = self.create_publisher(
            Range,
            f'{self.namespace}/uss_left/range',
            10
        )
        self.pub_right = self.create_publisher(
            Range,
            f'{self.namespace}/uss_right/range',
            10
        )

        self.get_logger().info(f'USS Republisher started')
        self.get_logger().info(f'  min_range: {self.min_range}')
        self.get_logger().info(f'  max_range: {self.max_range}')
        self.get_logger().info(f'  field_of_view: {self.field_of_view} rad ({self.field_of_view * 180 / 3.14159:.1f} deg)')
        self.get_logger().info(f'  radiation_type: {self.radiation_type} (0=ultrasound)')
        self.get_logger().info(f'  namespace: {self.namespace}')

    def callback_left(self, msg: Range):
        self.process_and_publish(msg, self.pub_left)

    def callback_right(self, msg: Range):
        self.process_and_publish(msg, self.pub_right)

    def process_and_publish(self, msg: Range, pub):
        # 填充缺失的字段
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.field_of_view = self.field_of_view
        msg.radiation_type = self.radiation_type
        
        # 发布修复后的消息
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UssRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
