# Livox SLAM System

## 项目概述

本项目是一个基于Livox Mid-360 LiDAR和Google Cartographer的SLAM（同时定位与地图构建）系统。该系统能够实时构建环境的3D地图，并提供定位功能。

## 功能特性

- 实时3D点云数据采集
- 高精度SLAM算法
- 数据录制与回放
- 实时可视化界面
- 完整的系统集成

## 前置要求

- Ubuntu 20.04或更高版本
- ROS 2 Foxy或更高版本
- Livox Mid-360 LiDAR
- 相关依赖包（见安装步骤）

## 安装步骤

1. 克隆仓库到ROS 2工作空间：
```bash
cd /public/github/livox_ws/src
git clone <repository_url>
```

2. 安装依赖：
```bash
cd /public/github/livox_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 构建项目：
```bash
colcon build --symlink-install
```

4. 源化环境：
```bash
source install/setup.bash
```

## 使用说明

详细使用说明请参考 [USAGE.md](USAGE.md) 和 [FULL_SYSTEM_USAGE.md](FULL_SYSTEM_USAGE.md) 文件。

### 基本启动流程

1. 构建工作空间：
```bash
cd /public/github/livox_ws
colcon build --symlink-install
source install/setup.bash
```

2. 启动完整SLAM系统：
```bash
ros2 launch livox_slam_system full_slam_system.launch.py
```

3. 查看建图结果：
```bash
ros2 run rviz2 rviz2 -d /public/github/livox_ws/src/livox_slam_system/config/livox_slam.rviz
```

### 单独启动组件

1. 启动Livox驱动：
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 启动Cartographer SLAM：
```bash
ros2 launch my_cartographer_launch cartographer_3d.launch.py \
  configuration_directory:=/public/github/livox_ws/src/livox_slam_system/config \
  configuration_basename:=livox_mid360_cartographer.lua
```

## 配置文件

- Livox配置文件：`config/mid360_config.json`
- Cartographer配置文件：`config/livox_mid360_cartographer.lua`
- RViz配置文件：`config/livox_slam.rviz`

## 数据录制

系统会自动录制以下话题到`/tmp/livox_slam_data`目录：
- `/livox/lidar` - 点云数据
- `/livox/imu` - IMU数据
- `/tf` 和 `/tf_static` - 坐标变换

## 查看建图结果

1. 实时查看：
```bash
ros2 run rviz2 rviz2 -d /public/github/livox_ws/src/livox_slam_system/config/livox_slam.rviz
```

2. 查看录制数据：
```bash
cd /tmp/livox_slam_data
ros2 bag play .
```

## 项目结构

```
livox_slam_system/
├── CMakeLists.txt
├── package.xml
├── setup.cfg
├── setup.py
├── README.md
├── USAGE.md
├── FULL_SYSTEM_USAGE.md
├── config/
│   ├── mid360_config.json
│   ├── livox_mid360_cartographer.lua
│   └── livox_slam.rviz
├── launch/
│   ├── simple_launch.py
│   ├── livox_slam_system.launch.py
│   └── full_slam_system.launch.py
├── resource/
│   └── livox_slam_system
├── scripts/
│   ├── data_recorder.py
│   ├── slam_manager.py
│   └── test_system.py
├── include/
│   └── livox_slam_system/
└── src/
```

## 贡献

欢迎提交Issue和Pull Request来改进本项目。

## 许可证

本项目采用Apache License 2.0许可证。