#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros
import tf_transformations
import math

class DynamicBaseFootprint(Node):
    def __init__(self):
        super().__init__('dynamic_base_footprint')
        
        # 参数
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')  # odom坐标系
        
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # TF广播器和监听器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器用于监听TF变换并发布
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz高频监听
        
        self.get_logger().info(f'动态base_footprint节点已启动')
        self.get_logger().info(f'odom帧: {self.odom_frame}')
        self.get_logger().info(f'base_link帧: {self.base_link_frame}')
        self.get_logger().info(f'base_footprint帧: {self.base_footprint_frame}')
    
    def quaternion_to_euler(self, q):
        """从四元数中提取完整的欧拉角"""
        # 使用tf_transformations库提取完整的欧拉角
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler  # 返回 (roll, pitch, yaw)
    
    def timer_callback(self):
        """监听odom->base_link的TF变换并发布base_link->base_footprint"""
        try:
            # 获取odom到base_link的变换
            odom_to_base_link = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_link_frame, 
                rclpy.time.Time()
            )
            
            # 创建base_link到base_footprint的变换
            base_link_to_footprint = TransformStamped()
            
            # 设置时间戳（与odom->base_link同步）
            base_link_to_footprint.header.stamp = odom_to_base_link.header.stamp
            base_link_to_footprint.header.frame_id = self.base_link_frame
            base_link_to_footprint.child_frame_id = self.base_footprint_frame
            
            # 设置变换：base_footprint在base_link下方，z坐标为0
            # 这意味着base_footprint相对于base_link的位置是向下的
            # 如果base_link在高度h处，那么base_footprint应该在高度0处
            # 所以变换应该是：x=0, y=0, z=-h
            
            # 获取base_link在odom坐标系中的高度
            base_link_height = odom_to_base_link.transform.translation.z
            
            # 设置变换：base_footprint在base_link下方base_link_height距离处
            base_link_to_footprint.transform.translation.x = 0.0
            base_link_to_footprint.transform.translation.y = 0.0
            base_link_to_footprint.transform.translation.z = -base_link_height  # 向下移动
            
            # 保持与base_link相同的yaw角，但roll和pitch取反以确保base_footprint与地面平行
            # 从base_link的旋转中提取完整的欧拉角
            q_odom_to_base_link = odom_to_base_link.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(q_odom_to_base_link)
            
            # roll和pitch取反，yaw保持不变
            # 这样base_footprint的朝向会与地面平行
            q_corrected = tf_transformations.quaternion_from_euler(-roll, -pitch, yaw)
            
            base_link_to_footprint.transform.rotation.x = q_corrected[0]
            base_link_to_footprint.transform.rotation.y = q_corrected[1]
            base_link_to_footprint.transform.rotation.z = q_corrected[2]
            base_link_to_footprint.transform.rotation.w = q_corrected[3]
            
            # 发布TF变换
            self.tf_broadcaster.sendTransform(base_link_to_footprint)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # TF变换不可用，暂时跳过
            pass
        except Exception as e:
            self.get_logger().error(f'发布TF变换时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = DynamicBaseFootprint()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()