#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import rosbag2_py
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # 初始化rosbag写入器
        self.writer = rosbag2_py.SequentialWriter()
        
        # 创建存储选项
        storage_options = rosbag2_py.StorageOptions(
            uri='/tmp/livox_data_recording',
            storage_id='sqlite3')
        
        # 创建转换选项
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        # 打开存储
        self.writer.open(storage_options, converter_options)
        
        # 创建话题元数据
        topic_info = rosbag2_py.TopicMetadata(
            name='/livox/lidar',
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr')
        
        # 创建话题
        self.writer.create_topic(topic_info)
        
        self.get_logger().info('Data recorder initialized')

    def listener_callback(self, msg):
        # 写入消息到rosbag
        self.writer.write(
            '/livox/lidar',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        self.get_logger().info('Recorded a point cloud message')

def main(args=None):
    rclpy.init(args=args)
    data_recorder = DataRecorder()
    rclpy.spin(data_recorder)
    
    # 销毁节点
    data_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()