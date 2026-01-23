#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros
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

    def quaternion_to_yaw(self, q):
        # 只返回 yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def timer_callback(self):
        """监听odom->base_link的TF变换并发布base_footprint"""
        try:
            # 获取odom到base_link的变换
            odom_to_base_link = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_link_frame, 
                rclpy.time.Time()
            )
            
            # 创建base_footprint到base_link的变换
            base_footprint_transform = odom_to_base_link
            
            # 设置时间戳（与odom->base_link同步）
            base_footprint_transform.child_frame_id = self.base_footprint_frame
            
            # 设置变换：base_footprint在base_link下方，z坐标为0
            # 保持相同的x,y位置和朝向，只改变z坐标
            base_footprint_transform.transform.translation.z = 0.0  # z坐标为0

            q = odom_to_base_link.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            q_new = tf_transformations.quaternion_from_euler(0, 0, yaw)
            base_footprint_transform.transform.rotation.x = q_new[0]
            base_footprint_transform.transform.rotation.y = q_new[1]
            base_footprint_transform.transform.rotation.z = q_new[2]
            base_footprint_transform.transform.rotation.w = q_new[3]
            
            # 发布TF变换
            self.tf_broadcaster.sendTransform(base_footprint_transform)
            
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