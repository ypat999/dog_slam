# 导航模块问题修复方案

## 问题描述
导航模块在Rviz中第一次给定目标点可以自主完成任务走过去，但是再给一个点就完全没反应了。

## 根本原因分析

### 1. 生命周期管理配置问题
- `nav2_params.yaml`中配置了`shutdown_on_error: true`，导致节点异常后全局停机
- 禁止自动重连机制，节点异常后无法自动恢复
- 超时时间设置过短（4秒），容易导致误判

### 2. 状态管理机制缺陷
- `send_goal_continuous.py`中`is_navigating`状态仅依赖`get_result_callback`重置
- 缺乏超时恢复机制，导航异常时状态无法自动重置
- 无异常检测机制，无法识别和处理卡死状态

### 3. 行为树配置问题
- 行为树缺少适当的状态重置逻辑
- 代价地图更新机制可能导致状态冲突

## 修复方案

### 1. 配置优化（已完成）
已修改`nav2_params.yaml`：
- `bond_timeout_ms`: 4000ms → 10000ms
- `attempt_respawn_reconnection`: false → true
- `respawn_reconnection_delay_s`: 0.0 → 2.0
- `shutdown_on_error`: true → false

### 2. 增强版导航脚本
创建`send_goal_continuous.py`改进版本：
- 添加导航超时机制（120秒）
- 实现状态自动恢复功能
- 增加状态检查定时器（每秒检查）
- 增强错误处理和重连机制
- 支持交互模式下的状态重置

### 3. 状态监控器
创建`navigation_monitor.py`：
- 实时监控导航状态
- 检测导航卡死情况
- 自动执行恢复操作
- 提供导航健康检查

### 4. 状态重置服务
创建`navigation_reset_service.py`：
- 提供`/reset_navigation_state`服务
- 支持手动重置导航状态
- 集成代价地图清理
- 生命周期节点重启

## 使用方法

### 1. 重新编译和安装
```bash
cd /home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
rm -rf build install log
colcon build --symlink-install
```

### 2. 启动导航状态监控器
```bash
# 在新终端中运行
ros2 run lio_sam navigation_monitor.py
```

### 3. 使用增强版导航脚本
```bash
# 交互模式（推荐）
ros2 run lio_sam send_goal_continuous.py

# 单次模式
ros2 run lio_sam send_goal_continuous.py 1.0 2.0
```

### 4. 手动重置导航状态
```bash
# 当导航出现问题时，调用重置服务
ros2 service call /reset_navigation_state std_srvs/srv/Empty
```

### 5. Web界面使用
Web界面无需修改，继续使用原有`nav2_web_control.html`：
- 点击地图设置目标点
- 如果第二次点击无反应，等待30秒自动恢复
- 或使用命令行调用重置服务

## 监控和调试

### 1. 查看导航状态
```bash
# 查看导航节点状态
ros2 node list | grep nav

# 查看导航话题
ros2 topic list | grep nav

# 查看导航服务
ros2 service list | grep nav
```

### 2. 日志监控
```bash
# 查看导航相关日志
ros2 launch lio_sam nav2.launch.py 2>&1 | grep -E "(nav|goal|pose)"

# 查看特定节点日志
ros2 launch lio_sam nav2.launch.py 2>&1 | grep -E "(bt_navigator|controller_server)"
```

### 3. 状态检查
```bash
# 检查生命周期管理器状态
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger

# 检查行为树状态
ros2 topic echo /behavior_tree_log
```

## 预期效果

1. **状态自动恢复**：导航卡死30秒后自动重置状态
2. **超时保护**：导航超时120秒后自动取消并重置
3. **增强稳定性**：生命周期管理器不再因错误而全局停机
4. **手动恢复**：提供手动重置导航状态的接口
5. **连续导航**：支持连续发送多个导航目标

## 故障排除

### 如果问题仍然存在
1. 检查所有节点是否正常运行：`ros2 node list`
2. 检查TF树是否完整：`ros2 run tf2_tools view_frames`
3. 检查代价地图是否正常更新：`ros2 topic hz /local_costmap/costmap`
4. 检查行为树日志：`ros2 launch lio_sam nav2.launch.py 2>&1 | grep -i error`

### 进一步诊断
1. 启用详细日志记录
2. 检查网络连接稳定性
3. 验证传感器数据流
4. 检查计算资源使用情况