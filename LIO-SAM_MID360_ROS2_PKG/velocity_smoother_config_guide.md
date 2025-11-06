# Velocity Smoother 配置指南

## feedback 参数说明

`feedback` 参数控制速度平滑器的反馈模式，影响速度命令的生成方式。

### 可选项

1. **"OPEN_LOOP"** (当前配置)
   - **描述**: 开环控制模式
   - **特点**: 
     - 不依赖里程计反馈
     - 直接根据目标速度生成平滑的速度命令
     - 计算简单，响应快速
     - 对里程计误差不敏感
   - **适用场景**: 
     - 里程计精度不高的情况
     - 需要快速响应的应用
     - 简单的导航任务

2. **"CLOSED_LOOP"** 
   - **描述**: 闭环控制模式
   - **特点**:
     - 使用里程计反馈进行速度控制
     - 可以更准确地跟踪期望速度
     - 能够补偿系统延迟和误差
     - 对里程计质量要求较高
   - **适用场景**:
     - 需要精确速度控制的应用
     - 里程计精度较高的系统
     - 复杂或精密的导航任务

## 其他相关参数

### 速度限制参数
```yaml
# 最大速度限制 [vx_max, vy_max, vtheta_max]
max_velocity: [0.5, 0.2, 5.0]

# 最小速度限制 [vx_min, vy_min, vtheta_min]  
min_velocity: [-0.05, -0.2, -5.0]
```

### 加速度限制参数
```yaml
# 最大加速度 [ax_max, ay_max, atheta_max]
max_accel: [4.0, 1.5, 5.0]

# 最大减速度 [ax_min, ay_min, atheta_min]
max_decel: [-2.5, -1.5, -5.0]
```

### 平滑参数
```yaml
# 平滑频率 (Hz)
smoothing_frequency: 20.0

# 是否缩放速度
scale_velocities: true

# 速度超时时间 (秒)
velocity_timeout: 5.0

# 死区速度设置 [vx, vy, vtheta]
deadband_velocity: [0.011, 0.12, 0.011]
```

### 减速参数
```yaml
# 开始减速的半径 (米)
slowdown_radius: 1.5

# 最大减速因子
max_slowdown_factor: 0.2
```

## 配置建议

### 对于直线+旋转导航策略
- **推荐**: 保持 `"OPEN_LOOP"` 模式
- **原因**: 开环模式响应更快，适合先直线移动后旋转的策略
- **注意**: 确保其他参数（如加速度、减速度）设置合理

### 对于精密导航
- **推荐**: 使用 `"CLOSED_LOOP"` 模式
- **原因**: 闭环控制可以更精确地跟踪期望轨迹
- **前提**: 确保里程计精度足够高

## 调试建议

1. **监控速度命令**: 使用 `rostopic echo /cmd_vel` 查看实际输出的速度命令
2. **检查平滑效果**: 观察速度曲线是否平滑，没有突变
3. **调整死区速度**: 如果机器人在低速时出现抖动，可以适当增加死区速度值
4. **优化减速参数**: 根据实际停止精度调整 `slowdown_radius` 和 `max_slowdown_factor`

## 相关文件
- 配置文件: `ros2/src/LIO-SAM_MID360_ROS2_DOG/config/nav2_params.yaml`
- 速度平滑器节点: `/controller_server`