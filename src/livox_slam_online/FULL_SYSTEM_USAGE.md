# 完整SLAM系统使用说明

## 系统概述

完整SLAM系统集成了以下组件：
1. Livox Mid-360 LiDAR驱动
2. Google Cartographer SLAM算法
3. 数据录制功能
4. 实时可视化界面

## 启动完整系统

### 1. 构建工作空间
```bash
cd /public/github/livox_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动完整SLAM系统
```bash
ros2 launch livox_slam_online full_slam_system.launch.py
```

这将同时启动：
- Livox Mid-360驱动节点
- Cartographer SLAM节点
- 数据录制节点

## 系统组件说明

### Livox驱动节点
- 发布点云数据到`/livox/lidar`话题
- 发布IMU数据到`/livox/imu`话题

### Cartographer SLAM节点
- 订阅`/livox/lidar`和`/livox/imu`话题
- 发布SLAM结果到相关话题
- 提供实时建图功能

### 数据录制节点
自动录制以下话题到`/tmp/livox_slam_data`目录：
- `/livox/lidar` - 点云数据
- `/livox/imu` - IMU数据
- `/tf` 和 `/tf_static` - 坐标变换
- `/scan_matched_points2` - 匹配后的点云
- `/submap_list` - 子图列表
- `/trajectory_node_list` - 轨迹节点列表

## 查看建图结果

### 实时可视化
```bash
ros2 run rviz2 rviz2 -d /public/github/dog_slam/src/livox_slam_online/config/livox_slam.rviz
```

### 查看录制数据
```bash
cd /tmp/livox_slam_data
ros2 bag play .
```

## 配置文件

### Livox配置
配置文件路径: `config/mid360_config.json`

### Cartographer配置
配置文件路径: `config/livox_mid360_cartographer.lua`

## 故障排除

### 1. 设备未连接
检查：
- USB连接线是否正常
- 设备是否供电
- 是否有权限访问设备

### 2. 点云数据未显示
检查：
- 设备是否正常启动
- 配置文件是否正确
- 网络连接是否正常

### 3. SLAM效果不佳
调整：
- 检查IMU数据是否正常
- 调整Cartographer参数
- 确保环境特征足够丰富

## 自定义开发

### 添加新的传感器
1. 修改配置文件以包含新传感器
2. 更新launch文件以启动新传感器节点
3. 调整Cartographer配置以融合新传感器数据

### 调整SLAM参数
1. 修改`config/livox_mid360_cartographer.lua`文件
2. 重新启动系统以应用更改

### 扩展功能
1. 添加新的数据处理节点
2. 实现自定义的SLAM后端
3. 集成其他传感器（如相机、GPS等）