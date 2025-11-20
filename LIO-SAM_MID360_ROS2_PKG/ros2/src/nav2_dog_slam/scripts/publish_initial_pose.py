#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import math


class InitialPosePublisher(Node):
    def __init__(self, x, y, yaw):
        super().__init__('initial_pose_publisher_for_slam_toolbox')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        # build message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        # covariance - reuse AMCL-like values
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.06853892326654787
        msg.pose.covariance = cov

        # publish once after small delay to ensure map & subscribers ready
        self.timer = self.create_timer(0.5, lambda: self._publish_once(msg))
        self._published = False

    def _publish_once(self, msg):
        if not self._published:
            self.pub.publish(msg)
            self.get_logger().info(f'Published initial pose to /initialpose: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y}, quat z={msg.pose.pose.orientation.z}, w={msg.pose.pose.orientation.w})')
            self._published = True
            # shutdown shortly after publishing
            self.create_timer(0.5, lambda: rclpy.shutdown())


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=33.5)
    parser.add_argument('--y', type=float, default=3.5)
    parser.add_argument('--yaw', type=float, default=1.57)
    args = parser.parse_args(argv)

    rclpy.init()
    node = InitialPosePublisher(args.x, args.y, args.yaw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
