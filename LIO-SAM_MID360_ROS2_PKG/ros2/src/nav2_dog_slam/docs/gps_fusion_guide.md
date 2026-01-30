# GPS融合功能使用指南

## 概述

本指南介绍如何使用ROS2的robot_localization模块将GPS信息融合到现有的LIO-SAM导航系统中。GPS融合可以显著提高机器人在室外环境中的全局定位精度和稳定性。

## 系统架构

GPS融合系统包含以下主要组件：

1. **NavSat Transform节点**：将GPS的经纬度坐标转换为地图坐标系
2. **EKF滤波器节点**：融合GPS数据与LIO-SAM里程计数据
3. **AMCL定位节点**：使用融合后的数据进行地图定位

## 文件说明

### 配置文件

- `config/gps_ekf.yaml` - EKF滤波器参数配置
- `config/navsat_transform.yaml` - GPS坐标转换参数配置

### Launch文件

- `launch/gps_fusion.launch.py` - 单独的GPS融合launch文件
- `launch/nav2_gps_fusion.launch.py` - 集成GPS融合的完整导航launch文件

### 测试脚本

- `scripts/test_gps_fusion.py` - GPS融合效果测试脚本

## 使用方法

### 方法1：使用集成导航系统（推荐）

```bash
# 启动完整的GPS融合导航系统
ros2 launch nav2_dog_slam nav2_gps_fusion.launch.py
```

### 方法2：单独启动GPS融合

```bash
# 单独启动GPS融合节点
ros2 launch nav2_dog_slam gps_fusion.launch.py

# 然后启动其他导航组件
ros2 launch nav2_dog_slam nav2_amcl.launch.py
```

### 方法3：测试GPS融合效果

```bash
# 启动GPS融合测试节点
ros2 run nav2_dog_slam test_gps_fusion.py
```

## 话题映射

### 输入话题

- `/gps/fix` - GPS原始数据 (sensor_msgs/NavSatFix)
- `/imu/data` - IMU数据 (sensor_msgs/Imu)
- `/Odometry` - LIO-SAM里程计数据 (nav_msgs/Odometry)

### 输出话题

- `/odometry/gps_fused` - GPS融合后的里程计数据
- `/tf` - 坐标变换
- `/tf_static` - 静态坐标变换

## 配置说明

### EKF滤波器配置

EKF滤波器配置在`gps_ekf.yaml`文件中，主要参数包括：

- **传感器配置**：定义哪些传感器数据用于融合
- **噪声协方差**：调整各传感器的权重
- **坐标系设置**：与现有系统保持一致

### NavSat Transform配置

GPS坐标转换配置在`navsat_transform.yaml`文件中：

- **坐标系映射**：定义GPS坐标系到地图坐标系的转换
- **参数调整**：根据实际GPS精度调整转换参数

## 性能优化建议

### GPS数据质量

1. **GPS天线位置**：确保GPS天线安装在开阔位置，避免遮挡
2. **数据频率**：建议GPS数据频率不低于10Hz
3. **精度要求**：使用RTK GPS可以获得厘米级定位精度

### 参数调优

1. **过程噪声**：根据GPS精度调整过程噪声协方差
2. **传感器权重**：根据传感器可靠性调整各传感器的权重
3. **数据延迟**：考虑GPS数据的传输延迟，适当调整时间同步

## 故障排除

### 常见问题

1. **GPS数据不接收**
   - 检查GPS设备连接
   - 确认GPS话题名称正确
   - 验证GPS数据格式

2. **融合效果不佳**
   - 调整EKF滤波器参数
   - 检查传感器时间同步
   - 验证坐标系转换正确性

3. **定位漂移**
   - 增加GPS权重
   - 检查IMU校准
   - 验证地图坐标系一致性

### 调试工具

使用以下工具进行调试：

```bash
# 查看GPS数据
ros2 topic echo /gps/fix

# 查看融合后的里程计
ros2 topic echo /odometry/gps_fused

# 查看坐标变换
ros2 run tf2_tools view_frames.py
```

## 性能指标

GPS融合系统应达到以下性能指标：

- **定位精度**：室外环境下优于1米（使用普通GPS）
- **定位稳定性**：位置方差比纯LIO-SAM降低30%以上
- **系统延迟**：融合延迟小于100ms

## 扩展功能

未来可以扩展的功能包括：

1. **多GPS融合**：支持多个GPS设备数据融合
2. **自适应滤波**：根据GPS信号质量动态调整滤波器参数
3. **地图匹配**：结合高精度地图进行更精确的定位

## 技术支持

如有问题，请检查以下资源：

- ROS2 robot_localization官方文档
- LIO-SAM项目文档
- 系统日志和诊断信息