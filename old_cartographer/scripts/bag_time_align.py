#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import os

# === 参数区 ===
input_bag = "/public/dataset/robot/livox_record/_cropped"
output_bag = "/public/dataset/robot/livox_record/_cropped_sync"
imu_topic = "/livox/imu"
imu_delay_sec = 1.0  # IMU 延迟秒数
remove_prefix_sec = 1.0  # 去除所有topic前1秒数据

# === 初始化 ===
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

# 获取元数据
info_reader = rosbag2_py.Info()
metadata = info_reader.read_metadata(input_bag, "sqlite3")
# 兼容不同 ROS2 版本的 topic 元数据结构
topics = {}
for t in metadata.topics_with_message_count:
    if hasattr(t, "topic_metadata"):  # 新版结构
        tm = t.topic_metadata
        topics[tm.name] = tm.type
    else:  # 旧版结构
        topics[t.name] = t.type

import time

# ROS2 旧版 (Foxy/Galactic) 兼容处理
if hasattr(metadata.starting_time, "time_since_epoch"):
    start_time_ns = metadata.starting_time.time_since_epoch().nanoseconds
else:
    # datetime 转换成纳秒时间戳
    start_time_ns = int(metadata.starting_time.timestamp() * 1e9)

remove_threshold_ns = start_time_ns + int(remove_prefix_sec * 1e9)

print(f"[INFO] Bag start time (ns): {start_time_ns}")
print(f"[INFO] Removing first {remove_prefix_sec}s of data before {remove_threshold_ns} ns")
print(f"[INFO] IMU topic '{imu_topic}' will be delayed by {imu_delay_sec}s")

# === 输出 writer ===
writer = rosbag2_py.SequentialWriter()
storage_options_out = rosbag2_py.StorageOptions(uri=output_bag, storage_id="sqlite3")
writer.open(storage_options_out, converter_options)

# 注册所有 topic
for topic, type_str in topics.items():
    msg_type = get_message(type_str)
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=topic,
        type=type_str,
        serialization_format="cdr"
    ))

# === 处理数据 ===
while reader.has_next():
    (topic, data, t) = reader.read_next()

    # 对 IMU 进行延迟
    if topic == imu_topic:
        # 若未定义 imu_queue 则初始化
        if 'imu_queue' not in locals():
            imu_queue = []
        t += int(imu_delay_sec * 1e9)
        imu_queue.append((topic, data, t))

    # 跳过前1秒的所有消息
    if t < remove_threshold_ns:
        continue

    # 每次只写入一个IMU数据
    if topic == imu_topic:
        if 'imu_queue' in locals() and len(imu_queue) > 0:
            imu_data = imu_queue.pop(0)  # 每次只取第一个
            writer.write(*imu_data)
    else:
        writer.write(topic, data, t)


print(f"[INFO] 新 bag 已保存至: {output_bag}")
