#!/usr/bin/env python3
"""
ROS Bag数据裁切脚本
用于裁切掉bag文件前两分钟的数据
"""

import os
import sys
import argparse
from datetime import datetime, timedelta
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.time import Duration

def crop_ros2_bag(input_bag_path, output_bag_path, crop_duration_sec=120):
    """
    裁切ROS2 bag文件，移除指定时间长度的初始数据
    
    Args:
        input_bag_path: 输入bag文件路径
        output_bag_path: 输出bag文件路径  
        crop_duration_sec: 要裁切的初始时间（秒）
    """
    print(f"开始裁切bag文件...")
    print(f"输入路径: {input_bag_path}")
    print(f"输出路径: {output_bag_path}")
    print(f"裁切时间: {crop_duration_sec}秒")
    
    # 创建读取器
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 获取话题信息
    topics = reader.get_all_topics_and_types()
    topic_names = [topic.name for topic in topics]
    topic_types = {topic.name: topic.type for topic in topics}
    
    print(f"发现话题: {topic_names}")
    
    # 创建写入器
    output_storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    output_converter_options = rosbag2_py.ConverterOptions('', '')
    writer = rosbag2_py.SequentialWriter()
    writer.open(output_storage_options, output_converter_options)
    
    # 注册话题
    for topic in topics:
        writer.create_topic(topic)
    
    # 查找起始时间
    first_timestamp = None
    crop_threshold = None
    message_count = 0
    cropped_count = 0
    
    # 先读取第一条消息获取起始时间
    while reader.has_next():
        topic_name, msg_data, timestamp = reader.read_next()
        if first_timestamp is None:
            first_timestamp = timestamp
            crop_threshold = first_timestamp + crop_duration_sec * 1e9  # 转换为纳秒
            print(f"起始时间: {first_timestamp}")
            print(f"裁切阈值: {crop_threshold}")
        
        # 只保留超过裁切阈值的消息
        if timestamp >= crop_threshold:
            writer.write(topic_name, msg_data, timestamp)
            message_count += 1
        else:
            cropped_count += 1
    
    print(f"原始消息总数: {message_count + cropped_count}")
    print(f"裁切消息数: {cropped_count}")
    print(f"保留消息数: {message_count}")
    print(f"裁切完成！")

def crop_ros1_bag(input_bag_path, output_bag_path, crop_duration_sec=120):
    """
    裁切ROS1 bag文件，移除指定时间长度的初始数据
    
    Args:
        input_bag_path: 输入bag文件路径
        output_bag_path: 输出bag文件路径
        crop_duration_sec: 要裁切的初始时间（秒）
    """
    try:
        import rosbag
    except ImportError:
        print("错误：未安装rosbag包，请安装ROS1的rosbag包")
        return
    
    print(f"开始裁切ROS1 bag文件...")
    print(f"输入路径: {input_bag_path}")
    print(f"输出路径: {output_bag_path}")
    print(f"裁切时间: {crop_duration_sec}秒")
    
    # 打开输入bag文件
    with rosbag.Bag(input_bag_path, 'r') as input_bag:
        # 获取起始时间
        start_time = input_bag.get_start_time()
        crop_time = start_time + crop_duration_sec
        
        print(f"起始时间: {datetime.fromtimestamp(start_time)}")
        print(f"裁切时间: {datetime.fromtimestamp(crop_time)}")
        
        # 创建输出bag文件
        with rosbag.Bag(output_bag_path, 'w') as output_bag:
            message_count = 0
            cropped_count = 0
            
            # 读取并过滤消息
            for topic, msg, timestamp in input_bag.read_messages():
                # 将时间戳转换为秒
                timestamp_sec = timestamp.to_sec()
                
                # 只保留超过裁切时间的数据
                if timestamp_sec >= crop_time:
                    output_bag.write(topic, msg, timestamp)
                    message_count += 1
                else:
                    cropped_count += 1
            
            print(f"原始消息总数: {message_count + cropped_count}")
            print(f"裁切消息数: {cropped_count}")
            print(f"保留消息数: {message_count}")
            print(f"裁切完成！")

def main():
    parser = argparse.ArgumentParser(description='裁切ROS bag文件的前几分钟数据')
    parser.add_argument('input_path', help='输入bag文件路径')
    parser.add_argument('-o', '--output', help='输出bag文件路径', default=None)
    parser.add_argument('-t', '--time', type=int, default=120, help='要裁切的初始时间（秒），默认120秒')
    parser.add_argument('--ros1', action='store_true', help='指定为ROS1 bag文件')
    parser.add_argument('--ros2', action='store_true', help='指定为ROS2 bag文件')
    
    args = parser.parse_args()
    
    # 自动生成输出文件名
    if args.output is None:
        base_name = os.path.splitext(args.input_path)[0]
        ext = '.bag' if args.ros1 else ''
        args.output = f"{base_name}_cropped{ext}"
    
    try:
        if args.ros1:
            crop_ros1_bag(args.input_path, args.output, args.time)
        elif args.ros2:
            crop_ros2_bag(args.input_path, args.output, args.time)
        else:
            # 自动检测bag类型
            if os.path.isdir(args.input_path):
                # ROS2 bag是一个目录
                crop_ros2_bag(args.input_path, args.output, args.time)
            else:
                # ROS1 bag是一个文件
                crop_ros1_bag(args.input_path, args.output, args.time)
                
    except Exception as e:
        print(f"错误：{e}")
        sys.exit(1)

if __name__ == '__main__':
    main()