#!/usr/bin/env python3

# 航点导航脚本
# 用于设置多个航点并依次导航

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import Header
import yaml
import sys
import time
from typing import List, Dict

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # 动作客户端
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # 等待动作服务器
        self.navigate_to_pose_client.wait_for_server()
        self.navigate_through_poses_client.wait_for_server()
        
        self.get_logger().info('航点导航器已启动')
    
    def create_pose_stamped(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> PoseStamped:
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position = Point()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 简单的偏航角转四元数
        pose.pose.orientation = Quaternion()
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(yaw)
        pose.pose.orientation.w = 1.0
        
        return pose
    
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """导航到单个目标点"""
        self.get_logger().info(f'导航到目标点: ({x}, {y}, {yaw})')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, yaw)
        
        # 发送目标
        future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('导航目标发送失败')
            return False
        
        goal_handle = future.result()
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result and result.result:
            self.get_logger().info('导航成功完成')
            return True
        else:
            self.get_logger().error('导航失败')
            return False
    
    def navigate_through_poses(self, waypoints: List[Dict[str, float]]) -> bool:
        """依次导航通过多个航点"""
        self.get_logger().info(f'开始多航点导航，共{len(waypoints)}个航点')
        
        poses = []
        for i, waypoint in enumerate(waypoints):
            pose = self.create_pose_stamped(
                waypoint['x'], 
                waypoint['y'], 
                waypoint.get('yaw', 0.0)
            )
            poses.append(pose)
            self.get_logger().info(f'航点{i+1}: ({waypoint["x"]}, {waypoint["y"]}, {waypoint.get("yaw", 0.0)})')
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        # 发送目标
        future = self.navigate_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('多航点导航目标发送失败')
            return False
        
        goal_handle = future.result()
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result and result.result:
            self.get_logger().info('多航点导航成功完成')
            return True
        else:
            self.get_logger().error('多航点导航失败')
            return False
    
    def load_waypoints_from_file(self, filename: str) -> List[Dict[str, float]]:
        """从YAML文件加载航点"""
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'加载航点文件失败: {e}')
            return []
    
    def save_waypoints_to_file(self, waypoints: List[Dict[str, float]], filename: str):
        """保存航点到YAML文件"""
        try:
            data = {'waypoints': waypoints}
            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f'航点已保存到 {filename}')
        except Exception as e:
            self.get_logger().error(f'保存航点文件失败: {e}')

def main():
    rclpy.init()
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  单点导航: python3 nav_waypoints.py single x y [yaw]")
        print("  多点导航: python3 nav_waypoints.py multi waypoint_file.yaml")
        print("  保存示例: python3 nav_waypoints.py save_example")
        return
    
    navigator = WaypointNavigator()
    
    try:
        if sys.argv[1] == 'single':
            # 单点导航
            if len(sys.argv) < 4:
                print("错误: 单点导航需要x和y坐标")
                return
            
            x = float(sys.argv[2])
            y = float(sys.argv[3])
            yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
            
            success = navigator.navigate_to_pose(x, y, yaw)
            
        elif sys.argv[1] == 'multi':
            # 多点导航
            if len(sys.argv) < 3:
                print("错误: 多点导航需要航点文件")
                return
            
            waypoint_file = sys.argv[2]
            waypoints = navigator.load_waypoints_from_file(waypoint_file)
            
            if waypoints:
                success = navigator.navigate_through_poses(waypoints)
            else:
                print("错误: 无法加载航点文件或文件为空")
                return
                
        elif sys.argv[1] == 'save_example':
            # 保存示例航点文件
            example_waypoints = [
                {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
                {'x': 2.0, 'y': 1.0, 'yaw': 1.57},
                {'x': 1.0, 'y': 2.0, 'yaw': 3.14},
                {'x': 0.0, 'y': 1.0, 'yaw': -1.57},
                {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            ]
            navigator.save_waypoints_to_file(example_waypoints, 'example_waypoints.yaml')
            print("示例航点文件已保存为 example_waypoints.yaml")
            return
            
        else:
            print(f"未知命令: {sys.argv[1]}")
            return
    
    except KeyboardInterrupt:
        navigator.get_logger().info('导航被取消')
    
    except Exception as e:
        navigator.get_logger().error(f'导航过程中发生错误: {e}')
    
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()