#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_ros
import tf_transformations
import math
import argparse

class DynamicBaseFootprint(Node):
    def __init__(self, base_link_frame, base_footprint_frame, odom_frame, use_sim_time):
        # 通过构造函数参数传递配置
        super().__init__('dynamic_base_footprint')
        
        self.base_link_frame = base_link_frame
        self.base_footprint_frame = base_footprint_frame
        self.odom_frame = odom_frame
        
        # 设置use_sim_time参数
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)])
        
        # TF广播器和监听器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅TF消息，在callback中处理变换
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            100
        )
        
        self.get_logger().info(f'动态base_footprint节点已启动')
        self.get_logger().info(f'odom帧: {self.odom_frame}')
        self.get_logger().info(f'base_link帧: {self.base_link_frame}')
        self.get_logger().info(f'base_footprint帧: {self.base_footprint_frame}')
        self.get_logger().info(f'use_sim_time: {use_sim_time}')
    
    def quaternion_to_euler(self, q):
        """从四元数中提取完整的欧拉角"""
        # 使用tf_transformations库提取完整的欧拉角
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler  # 返回 (roll, pitch, yaw)
    
    def tf_callback(self, msg):
        """处理TF消息，发布odom->base_footprint变换"""
        # TFMessage包含一个transforms数组，遍历所有变换
        for transform in msg.transforms:
            # 检查是否是odom到base_link的变换
            if transform.header.frame_id == self.odom_frame and transform.child_frame_id == self.base_link_frame:
                try:
                    # 创建odom到base_footprint的变换
                    odom_to_footprint = TransformStamped()
                    
                    # 设置时间戳（与odom->base_link同步）
                    odom_to_footprint.header.stamp = transform.header.stamp
                    odom_to_footprint.header.frame_id = self.odom_frame
                    odom_to_footprint.child_frame_id = self.base_footprint_frame
                    
                    # 设置变换：base_footprint在base_link的投影位置，但z坐标为0
                    # 保持相同的x,y位置，但z坐标投影到地面
                    
                    # 获取base_link在odom坐标系中的位置
                    base_link_x = transform.transform.translation.x
                    base_link_y = transform.transform.translation.y
                    base_link_z = transform.transform.translation.z
                    
                    # 设置变换：base_footprint在base_link的x,y位置，但z坐标为0
                    odom_to_footprint.transform.translation.x = base_link_x
                    odom_to_footprint.transform.translation.y = base_link_y
                    odom_to_footprint.transform.translation.z = base_link_z  # 地面高度
                    



                    # 保持与base_link相同的yaw角，但roll和pitch为0以确保base_footprint与地面平行
                    # 从base_link的旋转中提取完整的欧拉角
                    q_odom_to_base_link = transform.transform.rotation
                    # roll, pitch, yaw = self.quaternion_to_euler(q_odom_to_base_link)
                    
                    # 只保留yaw角，roll和pitch设为0（与地面平行）
                    # q_ground_parallel = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

                    yaw = math.atan2(
                        2.0 * (q_odom_to_base_link.w * q_odom_to_base_link.z + q_odom_to_base_link.x * q_odom_to_base_link.y),
                        1.0 - 2.0 * (q_odom_to_base_link.y ** 2 + q_odom_to_base_link.z ** 2)
                    )

                    q_yaw = [
                        math.cos(yaw * 0.5),
                        0.0,
                        0.0,
                        math.sin(yaw * 0.5)
                    ]


                    # 转为python
                    # double yaw = std::atan2(
                    #     2.0 * (q.w()*q.z() + q.x()*q.y()),
                    #     1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())
                    # );

                    # Eigen::Quaterniond q_yaw(
                    #     std::cos(yaw * 0.5),
                    #     0.0,
                    #     0.0,
                    #     std::sin(yaw * 0.5)
                    # );


            
                    odom_to_footprint.transform.rotation.w = q_yaw[0]
                    odom_to_footprint.transform.rotation.x = q_yaw[1]
                    odom_to_footprint.transform.rotation.y = q_yaw[2]
                    odom_to_footprint.transform.rotation.z = q_yaw[3]
                    
                    # 发布TF变换
                    self.tf_broadcaster.sendTransform(odom_to_footprint)
                    
                except Exception as e:
                    self.get_logger().error(f'发布TF变换时出错: {str(e)}')

def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Dynamic base_footprint TF publisher')
    parser.add_argument('--base_link_frame', default='base_link', help='Base link frame name')
    parser.add_argument('--base_footprint_frame', default='base_footprint', help='Base footprint frame name')
    parser.add_argument('--odom_frame', default='odom', help='Odom frame name')
    parser.add_argument('--use_sim_time', default='false', help='Use simulation time (true/false)')
    
    # 解析命令行参数
    parsed_args, unknown = parser.parse_known_args(args)
    
    # 转换use_sim_time为布尔值
    use_sim_time_bool = parsed_args.use_sim_time.lower() == 'true'
    
    rclpy.init(args=args)
    
    node = DynamicBaseFootprint(
        base_link_frame=parsed_args.base_link_frame,
        base_footprint_frame=parsed_args.base_footprint_frame,
        odom_frame=parsed_args.odom_frame,
        use_sim_time=use_sim_time_bool
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()