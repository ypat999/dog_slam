# My Cartographer Launch

这个包提供了一个完整的launch文件，用于运行Cartographer 3D SLAM，包含所有必要的TF变换和数据播放。

## 功能

- Cartographer 3D SLAM节点
- RViz2可视化
- 静态TF变换发布器 (map -> base_link 和 base_link -> livox_frame)
- Bag数据播放器

## 使用方法

### 基本使用

```bash
ros2 launch my_cartographer_launch cartographer_3d.launch.py
```

### 指定bag文件路径

```bash
ros2 launch my_cartographer_launch cartographer_3d.launch.py bag_path:=/path/to/your/bag/file
```

### 使用仿真时间

```bash
ros2 launch my_cartographer_launch cartographer_3d.launch.py use_sim_time:=true
```

## 参数说明

- `bag_path`: 要播放的bag文件路径，默认为 `/public/dataset/robot/Indoor_sampledata.bag`
- `use_sim_time`: 是否使用仿真时间，默认为 `false`

## 注意事项

1. 确保你的系统中存在指定路径的bag文件，或者通过 `bag_path` 参数指定正确的bag文件路径。
2. 如果要在自己的环境中使用，请确保已安装所有依赖项：
   - cartographer_ros
   - tf2_ros
   - rviz2

## Bag文件准备

如果你没有 `/public/dataset/robot/Indoor_sampledata.bag` 文件，你需要：
1. 获取或创建一个包含点云数据的bag文件
2. 在运行launch文件时通过 `bag_path` 参数指定正确的路径

例如：
```bash
ros2 launch my_cartographer_launch cartographer_3d.launch.py bag_path:=/home/user/my_data.bag
```