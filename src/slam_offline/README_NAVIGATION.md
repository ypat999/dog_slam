# Nav2 导航集成指南

本文档描述了如何在dog_slam项目中集成Nav2导航功能，支持在线SLAM、离线SLAM和使用已有地图三种模式。

## 功能概述

- **在线SLAM模式**: 实时使用Livox LiDAR进行SLAM建图和导航
- **离线SLAM模式**: 使用bag文件进行SLAM建图，同时支持导航
- **纯导航模式**: 使用已有的地图文件进行导航

## 依赖安装

确保已安装以下ROS2包：

```bash
# 安装Nav2相关包
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan

# 如果缺少其他依赖，请根据错误提示安装
```

## 使用方法

### 1. 完整SLAM+导航启动

#### 在线SLAM模式（实时建图+导航）
```bash
ros2 launch slam_offline slam_nav_bringup.launch.py mode:=online
```

#### 离线SLAM模式（bag文件建图+导航）
```bash
ros2 launch slam_offline slam_nav_bringup.launch.py mode:=offline bag_path:=/path/to/your/bag/
```

#### 纯导航模式（使用已有地图）
```bash
ros2 launch slam_offline slam_nav_bringup.launch.py mode:=nav_only map_yaml_file:=/path/to/map.yaml
```

### 2. 单独启动导航

如果SLAM已经在运行，可以单独启动导航：

```bash
# 在线SLAM导航
ros2 launch slam_offline nav2_navigation.launch.py slam_mode:=online

# 离线SLAM导航
ros2 launch slam_offline nav2_navigation.launch.py slam_mode:=offline

# 使用已有地图导航
ros2 launch slam_offline nav2_navigation.launch.py slam_mode:=none map:=/path/to/map.yaml
```

### 3. 地图保存

在SLAM模式下，可以保存当前地图：

```bash
# 保存地图
./src/slam_offline/scripts/save_map.sh my_map /path/to/output/directory/
```

### 4. 航点导航

#### 单点导航
```bash
# 导航到坐标(1.0, 2.0)，偏航角0.5弧度
python3 src/slam_offline/scripts/nav_waypoints.py single 1.0 2.0 0.5
```

#### 多点导航
```bash
# 使用航点文件进行多点导航
python3 src/slam_offline/scripts/nav_waypoints.py multi src/slam_offline/scripts/example_waypoints.yaml
```

#### 创建航点文件示例
```bash
# 生成示例航点文件
python3 src/slam_offline/scripts/nav_waypoints.py save_example
```

## 参数配置

### 主要启动参数

- `mode`: 运行模式 (online/offline/nav_only)
- `use_sim_time`: 是否使用仿真时间
- `bag_path`: bag文件路径（在线/离线模式）
- `map_yaml_file`: 地图文件路径（纯导航模式）
- `imu_time_offset`: IMU时间偏移

### Nav2参数配置

主要配置文件：`src/slam_offline/config/nav2_params.yaml`

- **AMCL参数**: 调整定位精度和粒子数量
- **代价地图参数**: 调整障碍物检测范围和膨胀半径
- **路径规划参数**: 调整路径规划算法和速度限制
- **恢复行为**: 配置机器人卡住时的恢复策略

## 坐标系说明

系统使用以下TF坐标系：

- `map`: 地图坐标系（固定）
- `odom`: 里程计坐标系
- `base_link`: 机器人基座坐标系
- `livox_frame`: LiDAR坐标系

## RViz界面

启动后自动打开RViz导航界面，包含：

- 地图显示
- 激光扫描数据
- 全局和局部路径
- 代价地图
- 导航工具栏

## 常见问题

### 1. 导航无法启动

- 检查TF变换是否正确发布
- 确认激光数据话题 `/scan` 是否正常
- 检查生命周期管理器状态

### 2. 定位漂移

- 调整AMCL参数中的粒子数量
- 检查IMU时间同步
- 确认激光扫描数据质量

### 3. 路径规划失败

- 检查代价地图参数
- 确认机器人footprint设置
- 调整膨胀半径参数

### 4. 航点导航失败

- 检查航点坐标是否在可行区域
- 确认导航栈已正确启动
- 查看导航日志获取详细信息

## 高级配置

### 自定义航点格式

航点YAML文件格式：
```yaml
waypoints:
  - x: 1.0
    y: 2.0
    yaw: 0.5  # 可选，默认为0
  - x: 3.0
    y: 4.0
```

### 代价地图调优

在`nav2_params.yaml`中调整：
- `inflation_radius`: 膨胀半径
- `cost_scaling_factor`: 代价值缩放因子
- `max_obstacle_height`: 最大障碍物高度

### 速度限制

调整控制器参数：
- `max_vel_x`: 最大线速度
- `max_vel_theta`: 最大角速度
- `acc_lim_x`: 线加速度限制

## 支持

如有问题，请查看相关日志或联系维护人员。