# ROS Bag数据裁切工具使用说明

## 功能
裁切掉ROS bag文件前两分钟（或指定时间）的初始数据。

## 使用方法

### 基本用法
```bash
# 裁切ROS2 bag数据（默认裁切120秒）
python3 crop_bag_data.py /public/dataset/robot/livox_record/

# 裁切ROS1 bag数据
python3 crop_bag_data.py /path/to/your/data.bag --ros1

# 指定裁切时间（秒）
python3 crop_bag_data.py /public/dataset/robot/livox_record/ --time 60

# 指定输出文件名
python3 crop_bag_data.py /public/dataset/robot/livox_record/ -o /path/to/output/
```

### 参数说明
- `input_path`: 输入bag文件路径（ROS1）或目录（ROS2）
- `-o, --output`: 输出文件路径（可选，默认自动生成）
- `-t, --time`: 裁切时间（秒），默认120秒
- `--ros1`: 指定为ROS1 bag文件
- `--ros2`: 指定为ROS2 bag文件

## 输出文件
- ROS1: 生成 `原文件名_cropped.bag`
- ROS2: 生成 `原目录名_cropped/` 目录

## 注意事项
1. 确保有足够的磁盘空间存储裁切后的数据
2. 对于大型bag文件，处理可能需要一些时间
3. 建议在处理前备份原始数据
4. ROS2 bag数据是一个目录，不是一个单独的文件

## 依赖
- Python3
- ROS2环境（包含rosbag2_py）
- 对于ROS1 bag文件，需要安装ROS1的rosbag包

## 示例
```bash
# 裁切livox_record数据的前2分钟
cd /public/github/dog_slam/scripts
python3 crop_bag_data.py /public/dataset/robot/livox_record/

# 裁切前1分钟的数据并指定输出位置
python3 crop_bag_data.py /public/dataset/robot/livox_record/ -t 60 -o /tmp/livox_record_cropped/
```