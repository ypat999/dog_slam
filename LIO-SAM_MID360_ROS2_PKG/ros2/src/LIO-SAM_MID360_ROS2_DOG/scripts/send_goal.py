#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

class GoalSender(Node):

    def __init__(self):
        super().__init__('goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置目标位置
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: Current pose: x={0:.2f}, y={1:.2f}'.format(
            feedback.current_pose.pose.position.x, 
            feedback.current_pose.pose.position.y))

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: send_goal.py <x> <y>")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    goal_sender = GoalSender()
    goal_sender.send_goal(x, y)
    
    try:
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        pass
    finally:
        goal_sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()