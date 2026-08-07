#!/usr/bin/env python3
"""交互式三维目标点/定位坐标发布器。

在 RViz 中用鼠标拖拽箭头，松开时（MOUSE_UP）按 output_type 发布消息：
  - point         -> geometry_msgs/PointStamped              (OctoPlanner3D 的 clicked_point 目标点)
  - pose          -> geometry_msgs/PoseWithCovarianceStamped (lidar_localization 的 initialpose 定位坐标)
  - pose_stamped  -> geometry_msgs/PoseStamped               (兼容原始 /goal_pose_3d 用法)

参数：
  frame_id      坐标系，默认 map
  output_topic  输出话题
  output_type   point / pose / pose_stamped
  marker_name   InteractiveMarkerServer 命名空间（多实例时需唯一）
  initial_x/y/z marker 初始位置
  marker_scale  marker 缩放
"""
import math

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback,
)
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Pose


class GoalPoseMarker(Node):
    def __init__(self):
        super().__init__('goal_pose_marker')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('output_topic', '/goal_pose_3d')
        self.declare_parameter('output_type', 'point')
        self.declare_parameter('marker_name', 'goal_marker')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 1.0)
        self.declare_parameter('marker_scale', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        output_topic = self.get_parameter('output_topic').value
        self.output_type = self.get_parameter('output_type').value
        marker_name = self.get_parameter('marker_name').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_z = self.get_parameter('initial_z').value
        self.marker_scale = self.get_parameter('marker_scale').value

        # 按输出类型创建发布器
        if self.output_type == 'point':
            self.pub = self.create_publisher(PointStamped, output_topic, 10)
        elif self.output_type == 'pose':
            self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        elif self.output_type == 'pose_stamped':
            self.pub = self.create_publisher(PoseStamped, output_topic, 10)
        else:
            raise ValueError(f'不支持的 output_type: {self.output_type}')

        # 记录上一次的位置和方向
        self.last_position = None
        self.last_yaw = 0.0

        self.server = InteractiveMarkerServer(self, marker_name)
        self.create_marker()

        self.get_logger().info(
            f'Ready. 拖拽 marker "{marker_name}" 以发布 {self.output_type} -> {output_topic} (frame: {self.frame_id})')

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = 'goal'
        int_marker.scale = self.marker_scale

        # 设置初始位置
        int_marker.pose.position.x = self.initial_x
        int_marker.pose.position.y = self.initial_y
        int_marker.pose.position.z = self.initial_z
        int_marker.pose.orientation.w = 1.0

        # 创建箭头marker来显示方向
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 1.0  # 箭头长度
        marker.scale.y = 0.2  # 箭头宽度
        marker.scale.z = 0.2  # 箭头高度
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.name = 'move_3d'
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True
        control.markers.append(marker)

        int_marker.controls.append(control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        # 获取当前位置
        current_position = [
            feedback.pose.position.x,
            feedback.pose.position.y,
            feedback.pose.position.z,
        ]

        # 计算yaw方向（基于移动方向）
        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]

            # 有显著移动时根据移动方向更新偏航角，否则保持上一次
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                yaw = math.atan2(dy, dx)
                if yaw > math.pi:
                    yaw -= 2 * math.pi
                elif yaw < -math.pi:
                    yaw += 2 * math.pi
                self.last_yaw = yaw
            else:
                yaw = self.last_yaw
        else:
            yaw = self.last_yaw

        self.last_position = current_position

        # 更新marker的方向显示
        self.update_marker_orientation(yaw)

        # 发布目标点
        self.publish_output(current_position, yaw)

        self.get_logger().info(
            f'目标点更新: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f}), '
            f'偏航角: {math.degrees(yaw):.1f} deg -> {self.get_topic_name()}')

    def get_topic_name(self):
        return self.pub.topic_name

    def publish_output(self, position, yaw):
        stamp = self.get_clock().now().to_msg()

        if self.output_type == 'point':
            msg = PointStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            msg.point.x = position[0]
            msg.point.y = position[1]
            msg.point.z = position[2]
        elif self.output_type == 'pose':
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            msg.pose.pose.position.x = position[0]
            msg.pose.pose.position.y = position[1]
            msg.pose.pose.position.z = position[2]
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.pose.covariance = [0.0] * 36
        else:  # pose_stamped
            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(msg)

    def update_marker_orientation(self, yaw):
        """更新marker的方向显示"""
        pose = Pose()

        # 使用当前位置（如果存在）或初始位置
        if self.last_position is not None:
            pose.position.x = self.last_position[0]
            pose.position.y = self.last_position[1]
            pose.position.z = self.last_position[2]
        else:
            pose.position.x = self.initial_x
            pose.position.y = self.initial_y
            pose.position.z = self.initial_z

        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        self.server.setPose('goal', pose)
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
