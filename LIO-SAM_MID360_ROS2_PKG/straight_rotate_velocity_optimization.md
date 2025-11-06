# 直线+旋转导航策略的速度平滑器优化建议

## 当前配置分析

当前`feedback: "OPEN_LOOP"`配置对于直线+旋转导航策略是合适的，因为：
- 开环模式响应更快，适合分段导航策略
- 减少了对里程计精度的依赖
- 简化了控制逻辑

## 针对直线+旋转策略的优化建议

### 1. 反馈模式建议
```yaml
# 保持当前配置
feedback: "OPEN_LOOP"
```

### 2. 速度限制优化
对于直线+旋转策略，建议调整以下参数：

```yaml
# 优化建议 - 直线移动阶段
max_velocity: [0.5, 0.1, 2.0]  # 降低横向和旋转速度，提高直线稳定性
min_velocity: [-0.05, -0.1, -2.0]

# 加速度优化
max_accel: [3.0, 1.0, 3.0]    # 降低加速度，提高平稳性
max_decel: [-2.0, -1.0, -3.0]
```

### 3. 平滑参数调整
```yaml
# 提高平滑频率以获得更平滑的速度曲线
smoothing_frequency: 30.0  # 从20Hz提高到30Hz

# 调整死区速度避免低速抖动
deadband_velocity: [0.01, 0.05, 0.01]  # 降低横向死区
```

### 4. 减速参数优化
```yaml
# 调整减速参数配合目标检查器
slowdown_radius: 0.8      # 减小减速半径，配合0.15m的位置容差
max_slowdown_factor: 0.3   # 增加减速因子，提高停止精度
```

## 分阶段速度控制策略

### 阶段1：直线移动到位置
- 主要使用X轴速度
- 限制Y轴和旋转速度以减少干扰
- 建议参数：
```yaml
# 临时调整（可通过动态参数服务）
max_velocity: [0.5, 0.05, 0.5]  # 大幅降低横向和旋转速度
```

### 阶段2：原地旋转对齐
- 主要使用旋转速度
- X、Y轴速度保持为零
- 建议参数：
```yaml
# 临时调整（可通过动态参数服务）
max_velocity: [0.0, 0.0, 2.0]   # 仅允许旋转
```

## 动态参数调整方案

### 使用参数服务进行动态调整
```bash
# 阶段1：直线移动模式
ros2 param set /velocity_smoother max_velocity [0.5, 0.05, 0.5]

# 阶段2：旋转模式  
ros2 param set /velocity_smoother max_velocity [0.0, 0.0, 2.0]

# 恢复默认
ros2 param set /velocity_smoother max_velocity [0.5, 0.2, 5.0]
```

## 监控和调试建议

### 1. 速度曲线监控
```bash
# 监控实际速度输出
ros2 topic echo /cmd_vel

# 监控平滑后的速度
ros2 topic echo /velocity_smoother/smoothed_cmd_vel
```

### 2. 性能评估指标
- **直线移动精度**：位置误差 < 0.15m
- **旋转精度**：角度误差 < 0.15rad
- **速度平滑性**：无突变和震荡
- **响应时间**：目标切换响应 < 1s

### 3. 调优步骤
1. 先调整`max_velocity`限制
2. 然后优化`max_accel/decel` 
3. 最后微调`deadband_velocity`
4. 根据实际效果调整`slowdown`参数

## 故障排除

### 常见问题

1. **直线移动不稳定**
   - 检查Y轴速度限制是否过大
   - 降低`vy_std`采样标准差
   - 增加`PreferForwardCritic`权重

2. **旋转过冲**
   - 降低旋转最大速度
   - 增加旋转减速度
   - 调整`Spin`行为的`spin_dist`参数

3. **停止精度差**
   - 减小`slowdown_radius`
   - 增加`max_slowdown_factor`
   - 调整目标检查器的`xy_goal_tolerance`

## 相关命令

```bash
# 查看当前参数
ros2 param get /velocity_smoother feedback
ros2 param get /velocity_smoother max_velocity

# 动态调整参数
ros2 param set /velocity_smoother feedback "OPEN_LOOP"
ros2 param set /velocity_smoother max_velocity [0.5, 0.1, 2.0]

# 监控性能
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel --no-arr
```

## 配置文件位置
- 主配置文件：`ros2/src/LIO-SAM_MID360_ROS2_DOG/config/nav2_params.yaml`
- 速度平滑器配置：第795行附近