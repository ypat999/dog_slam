# 使用说明

## 系统功能

本系统集成了以下功能：
1. 实时连接Livox Mid-360激光雷达
2. 在ROS2中发布点云数据
3. 录制所有传感器数据到rosbag2文件供后续调试
4. 使用Cartographer进行SLAM建图

## 启动系统

### 1. 构建工作空间

```bash
cd /public/github/livox_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动完整系统

```bash
ros2 launch livox_slam_online livox_slam_online.launch.py
```

该命令将启动：
- Livox Mid-360驱动程序
- 数据录制到 `/tmp/livox_slam_data`

### 3. 单独启动Cartographer SLAM

由于之前构建时遇到了一些依赖问题，Cartographer部分需要手动启动：

```bash
ros2 launch slam_offline cartographer_3d.launch.py \
  configuration_directory:=/public/github/dog_slam/src/livox_slam_online/config \
  configuration_basename:=livox_mid360_cartographer.lua
```

## 配置文件说明

### Livox Mid-360配置
文件路径：`config/mid360_config.json`

主要配置项：
- `ip`: Livox Mid-360的IP地址（默认为192.168.1.150）
- `pcl_data_type`: 点云数据类型
- `extrinsic_parameter`: 外参配置（旋转和平移）

### Cartographer配置
文件路径：`config/livox_mid360_cartographer.lua`

主要配置项：
- `min_range` 和 `max_range`: 激光雷达有效距离范围
- `submaps.num_range_data`: 每个子地图的扫描数量
- `submaps.grid.high_resolution`: 高分辨率网格大小（米）
- `submaps.grid.low_resolution`: 低分辨率网格大小（米）

## 数据录制

系统会自动录制以下话题到 `/tmp/livox_slam_data`：
- `/livox/lidar`: 点云数据
- `/livox/imu`: IMU数据（如果可用）
- `/tf`: 坐标变换
- `/tf_static`: 静态坐标变换

播放录制的数据：
```bash
ros2 bag play /tmp/livox_slam_data
```

## 查看建图结果

启动RViz查看建图结果：
```bash
rviz2 -d /public/github/dog_slam/src/slam_offline/config/cartographer.rviz
```

## 故障排除

### 1. 连接不上Livox Mid-360
- 检查LiDAR是否正确连接电源和网线
- 确认LiDAR的IP地址与配置文件中的一致
- 确保电脑与LiDAR在同一网络中

### 2. 没有点云数据显示
- 检查LiDAR是否正常旋转
- 确认ROS2话题是否正确发布：`ros2 topic list | grep livox`

### 3. 建图质量不佳
- 检查环境光照条件（避免强光直射）
- 调整LiDAR的安装角度
- 修改Cartographer配置参数

## 自定义开发

### 添加新的数据处理节点

1. 在 `scripts/` 目录下创建新的Python脚本
2. 在 `setup.py` 中添加新的入口点
3. 在launch文件中添加节点启动配置

### 修改配置参数

根据实际使用场景，可以调整：
- LiDAR的外参配置
- Cartographer的建图参数
- 数据录制的话题列表