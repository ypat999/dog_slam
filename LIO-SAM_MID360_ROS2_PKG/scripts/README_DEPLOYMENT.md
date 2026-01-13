# LIO-SAM MID360 ROS2 开发板部署指南

## 概述

本部署脚本用于在全新的开发板环境中安装和配置LIO-SAM MID360 ROS2项目，包含完整的SLAM建图、定位和导航功能。

## 系统要求

- **操作系统**: Ubuntu 22.04 (Jammy Jellyfish) 推荐
- **硬件要求**: 
  - 至少10GB可用磁盘空间
  - 支持ROS2 Humble的硬件架构
  - 可选: Livox MID360激光雷达设备

## 部署脚本功能

### 主要功能模块

1. **系统环境检查和配置**
   - 操作系统版本验证
   - 磁盘空间检查
   - 基础依赖包安装

2. **ROS2 Humble安装**
   - 使用阿里云镜像源加速下载
   - 完整桌面版ROS2安装
   - 开发工具配置

3. **Livox-SDK2安装**
   - 激光雷达驱动库编译安装
   - 依赖库配置

4. **项目依赖包安装**
   - PCL点云库相关依赖
   - ROS2导航系统包
   - LIO-SAM特定依赖
   - GTSAM优化库
   - SLAM建图工具包（slam_toolbox、explore_lite）

5. **Gazebo仿真环境（可选）**
   - Gazebo Harmonic安装
   - 仿真模型下载

6. **项目构建**
   - Livox ROS2驱动构建
   - 完整ROS2工作空间构建
   - 依赖关系解析

7. **系统服务安装**
   - 导航系统服务配置（默认启用）
   - 手动建图服务配置
   - 自动探索建图服务配置
   - 用户权限设置

8. **环境配置**
   - 环境变量设置（MANUAL_BUILD_MAP、AUTO_BUILD_MAP）
   - udev设备规则配置
   - bashrc自动加载

9. **安装验证**
   - 环境加载测试
   - 关键包验证
   - 建图工具验证

## 使用方法

### 快速部署

```bash
# 下载脚本（如果尚未下载）
cd /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts

# 赋予执行权限
chmod +x deploy_new_board.sh

# 执行部署脚本（需要root权限）
sudo ./deploy_new_board.sh
```

### 分步部署（可选）

如果需要分步执行，可以修改脚本中的相应部分，或使用以下命令单独执行各步骤：

```bash
# 仅安装ROS2
sudo bash -c 'source deploy_new_board.sh && 安装ROS2部分'

# 仅构建项目
sudo bash -c 'source deploy_new_board.sh && 构建项目部分'
```

## 部署后操作

### 1. 重启系统
部署完成后建议重启系统以应用所有更改：
```bash
sudo reboot
```

### 2. 验证安装
重启后验证安装是否成功：
```bash
# 检查服务状态
sudo systemctl status lio_nav2_unified

# 手动测试环境
source ~/.lio_nav2_env.sh
ros2 pkg list | grep -E "(fast_lio|lio_sam|nav2)"
```

### 3. 启动服务

#### 自动启动（推荐）
服务已配置为系统启动时自动运行：
```bash
# 手动启动（如果需要）
sudo systemctl start lio_nav2_unified

# 查看服务状态
sudo systemctl status lio_nav2_unified

# 查看实时日志
sudo journalctl -u lio_nav2_unified -f
```

#### 手动启动
```bash
# 打开新终端
source ~/.lio_nav2_env.sh

# 启动统一导航系统（使用FAST-LIO算法）
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py slam_algorithm:=fast_lio

# 或使用其他算法
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py slam_algorithm:=lio_sam
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py slam_algorithm:=point_lio
```

## 支持的SLAM算法

- **FAST-LIO2** (推荐): 基于迭代卡尔曼滤波，实时性强
- **LIO-SAM**: 基于因子图优化，回环检测能力强
- **Point-LIO**: 基于点的激光惯性里程计
- **Faster-LIO**: 优化的快速激光惯性里程计
- **DLIO**: 直接激光惯性里程计

## 故障排除

### 常见问题

1. **ROS2环境加载失败**
   ```bash
   # 检查ROS2安装
   source /opt/ros/humble/setup.bash
   echo $ROS_DISTRO
   ```

2. **服务启动失败**
   ```bash
   # 查看详细日志
   sudo journalctl -u lio_nav2_unified -n 50
   
   # 重新加载服务
   sudo systemctl daemon-reload
   sudo systemctl restart lio_nav2_unified
   ```

3. **权限问题**
   ```bash
   # 检查用户组
   groups $(whoami)
   
   # 添加用户到必要组
   sudo usermod -aG dialout $(whoami)
   sudo usermod -aG audio $(whoami)
   ```

4. **构建失败**
   ```bash
   # 清理并重新构建
   cd /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
   rm -rf build install log
   colcon build --symlink-install
   ```

### 日志文件位置

- 系统服务日志: `/var/log/syslog`
- 服务特定日志: `journalctl -u lio_nav2_unified`
- ROS2日志: `~/.ros/log/`

## 硬件连接

### Livox MID360连接

1. 连接USB-C数据线
2. 检查设备识别:
   ```bash
   lsusb | grep -i livox
   dmesg | grep -i livox
   ```
3. 验证设备权限

### 网络配置（多机通信）

如需多机通信，配置网络环境：
```bash
# 设置ROS2域ID
export ROS_DOMAIN_ID=27

# 或设置特定IP
export ROS_IP=192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311
```

## 性能优化建议

1. **构建优化**
   ```bash
   # 使用多核编译
   colcon build --symlink-install --parallel-workers $(nproc)
   ```

2. **内存优化**
   - 关闭不必要的图形界面
   - 优化ROS2参数配置

3. **网络优化**
   - 使用有线网络连接
   - 配置合适的ROS域ID

## 联系支持

如遇问题，请检查：
1. 系统日志和ROS2日志
2. 硬件连接状态
3. 网络配置
4. 权限设置

或参考项目主README文档获取更多信息。