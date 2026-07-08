#!/usr/bin/env python3
"""
OctoPlanner3D Nav2 Adapter
适配OctoPlanner3D（订阅goal_pose话题）与Nav2行为树（期望ComputePathToPose服务）的接口

工作流程：
1. Nav2 bt_navigator通过ComputePathToPose服务请求路径
2. 本适配节点收到请求后，发布goal_pose给OctoPlanner
3. 等待OctoPlanner发布planned_path
4. 将路径返回给Nav2

这样Nav2的行为树可以正常工作，而不需要修改OctoPlanner的代码。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

import time
from threading import Lock


class OctoPlannerNav2Adapter(Node):
    def __init__(self):
        super().__init__('octoplanner_nav2_adapter')

        # 参数
        self.planner_id = self.declare_parameter('planner_id', 'GridBased').value
        self.wait_timeout = self.declare_parameter('wait_timeout', 5.0).value

        # 回调组（允许多线程并发）
        self.callback_group = ReentrantCallbackGroup()

        # 发布goal_pose给OctoPlanner
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)

        # 订阅OctoPlanner的路径
        self.path_sub = self.create_subscription(
            Path, 'planned_path', self.path_callback, 10,
            callback_group=self.callback_group)

        # Nav2 ComputePathToPose Action Server
        self.action_server = ActionServer(
            self, ComputePathToPose, 'compute_path_to_pose',
            self.execute_callback,
            callback_group=self.callback_group)

        # 路径缓存和锁
        self.latest_path = None
        self.path_lock = Lock()
        self.path_received = False

        self.get_logger().info(
            f'OctoPlanner Nav2 Adapter initialized. '
            f'Planner ID: {self.planner_id}, Timeout: {self.wait_timeout}s')

    def path_callback(self, msg: Path):
        """接收OctoPlanner发布的路径"""
        with self.path_lock:
            self.latest_path = msg
            self.path_received = True
            self.get_logger().info(
                f'Received path from OctoPlanner: {len(msg.poses)} poses')

    def execute_callback(self, goal_handle):
        """处理Nav2的ComputePathToPose请求"""
        self.get_logger().info('Received ComputePathToPose request')

        # 清空之前的路径缓存
        with self.path_lock:
            self.latest_path = None
            self.path_received = False

        # 构造goal_pose消息发布给OctoPlanner
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.header.frame_id = goal_handle.request.goal.header.frame_id
        goal_pose_msg.pose = goal_handle.request.goal.pose

        self.get_logger().info(
            f'Publishing goal_pose to OctoPlanner: '
            f'[{goal_pose_msg.pose.position.x:.3f}, '
            f'{goal_pose_msg.pose.position.y:.3f}, '
            f'{goal_pose_msg.pose.position.z:.3f}]')

        self.goal_pub.publish(goal_pose_msg)

        # 等待OctoPlanner发布路径
        start_time = time.time()
        while time.time() - start_time < self.wait_timeout:
            with self.path_lock:
                if self.path_received and self.latest_path is not None:
                    break
            time.sleep(0.1)

        # 检查是否收到路径
        with self.path_lock:
            if self.latest_path is None:
                self.get_logger().error(
                    f'Timeout waiting for OctoPlanner path ({self.wait_timeout}s)')
                goal_handle.abort()
                result = ComputePathToPose.Result()
                result.path = Path()  # 空路径
                return result

            # 返回路径给Nav2
            result = ComputePathToPose.Result()
            result.path = self.latest_path

            self.get_logger().info(
                f'Returning path to Nav2: {len(result.path.poses)} poses')

            goal_handle.succeed()
            return result


def main(args=None):
    rclpy.init(args=args)

    adapter_node = OctoPlannerNav2Adapter()

    # 使用多线程执行器支持并发Action处理
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(adapter_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        adapter_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()