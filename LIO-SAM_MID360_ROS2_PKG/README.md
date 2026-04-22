# FAST-LIO2 & LIO-SAM MID360 ROS2 Package

基于ROS2 Humble的Livox MID360激光雷达SLAM与导航系统，集成了多种先进的SLAM算法和导航功能。

## 主要特性

- **多SLAM算法支持**: FAST-LIO2, LIO-SAM, Point-LIO, Super-LIO
- **统一导航系统**: 集成Nav2导航框架，支持自主探索和路径规划
- **多机器人支持**: 通过 namespace 实现多个导航系统隔离
- **实时建图**: 支持在线建图和离线地图构建
- **稳定导航**: 优化base_footprint，实现高速稳定导航
- **Gazebo Garden仿真**: 支持Gazebo Garden GPU加速仿真，提供多种仿真环境

## 系统要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS2 Humble
- **硬件**: Livox MID360激光雷达
- **依赖库**: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

### 推荐使用阿里源：
```bash
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu/ jammy main
deb [arch=amd64] https://mirrors.aliyun.com/ubuntu/ jammy main

# 添加gpg key
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## 依赖项安装

### Livox-SDK2 安装

```bash
# 安装依赖项
sudo apt-get update
sudo apt-get install -y git cmake g++ libboost-all-dev libpcl-dev

# 克隆并编译Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake ..
make
sudo make install
```

### ROS2 依赖包

```bash
# SLAM相关依赖
sudo apt install -y ros-humble-perception-pcl ros-humble-pcl-msgs \
    ros-humble-vision-opencv ros-humble-xacro ros-humble-vision-msgs

# GTSAM (LIO-SAM依赖)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# Super-LIO依赖
sudo apt install libgoogle-glog-dev libtbb-dev

# Nav2导航系统
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-server 
sudo apt-get update && sudo apt-get install -y \
    ros-humble-dwb-critics \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-controller \
    ros-humble-nav2-amcl \
    ros-humble-nav2-planner \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-waypoint-follower \
    ros-humble-rosbridge-server

# 点云转激光扫描
sudo apt install ros-humble-pointcloud-to-laserscan

# OctoMap建图
sudo apt install ros-humble-octomap ros-humble-octomap-msgs
sudo apt install ros-humble-octomap-server
```


## rqt
sudo apt install ros-humble-rqt  

## slam_toolbox
apt install ros-humble-slam-toolbox


# gazebo
sudo apt update
sudo apt install lsb-release curl gnupg

# 添加 GZ Sim GPG key
curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /etc/apt/trusted.gpg.d/gazebo.gpg > /dev/null

# 添加 Harmonic 软件源（Ubuntu 22.04 用 jammy）
echo "deb http://packages.osrfoundation.org/gz-harmonic/ubuntu `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/gz-harmonic.list > /dev/null

sudo apt update
sudo apt install mesa-vulkan-drivers vulkan-tools -y


sudo apt install \
  ros-humble-gazebo-*          \
  <!-- sudo apt install ros-humble-ros-gz
  sudo apt install gz-harmonic -->

  ros-humble-ros2-control* \    
  ros-humble-robot-state-publisher \ 
  ros-humble-joint-state-publisher \ 
  ros-humble-xacro  

#############################################
# 若Gazebo缺少基础模型，可手动克隆官方模型库
#############################################
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models


sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*

## 构建项目

#预留，不用这句 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

### 首次构建
```bash
# 删除旧构建文件
rm -rf build/ install/ log/

# 运行构建脚本
./build_ros2.sh
```

## 运行方式

现在系统使用统一的启动文件，不再支持单独启动各组件。统一入口集成了所有SLAM算法和导航功能，提供了更简洁、更稳定的使用体验。

### 统一启动方式（唯一入口）

使用统一的启动文件，支持多种SLAM算法和导航模式：

```bash
# 进入项目根目录
cd /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG

# 启动统一SLAM导航系统
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py

# 通过环境变量选择SLAM算法（默认使用FAST-LIO2）
# export SLAM_ALGORITHM=fast_lio    # 使用FAST-LIO2
# export SLAM_ALGORITHM=point_lio   # 使用Point-LIO
# export SLAM_ALGORITHM=super_lio   # 使用Super-LIO
# export SLAM_ALGORITHM=lio_sam     # 使用LIO-SAM
```

### 统一启动文件说明

统一启动文件 `lio_nav2_unified.launch.py` 位于 `nav2_dog_slam/launch/` 目录下，它集成了：

- **Livox MID360雷达驱动**：自动启动雷达驱动，无需单独启动
- **SLAM算法**：根据环境变量选择不同的SLAM算法（FAST-LIO2、Point-LIO、Super-LIO、LIO-SAM）
- **导航系统**：集成Nav2导航框架，支持路径规划和避障
- **点云转激光扫描**：自动将3D点云转换为2D激光扫描数据，供导航使用
- **TF变换管理**：统一管理所有坐标系变换，确保系统稳定性
- **Web控制界面**：启动rosbridge_websocket，支持通过浏览器控制机器人
# 多机器人（带 namespace）
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py ns:=rkbot
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py ns:=rkbot2
```

### SLAM算法选择

通过环境变量选择SLAM算法（默认使用Super-LIO）：

### 环境变量配置

通过设置环境变量，可以自定义系统行为：

```bash
# 选择SLAM算法
export SLAM_ALGORITHM=fast_lio    # 默认值，使用FAST-LIO2

# 建图模式
export MAP_BUILDING_MODE=true      # 启用建图模式

# 导航模式
export NAVIGATION_MODE=true        # 启用导航模式

# 启动系统
ros2 launch nav2_dog_slam lio_nav2_unified.launch.py
```

## 导航功能

### 自主探索
```bash
# 启动自主探索
ros2 launch m-explore explore.launch.py
```

### 路径规划与导航
系统支持Nav2的完整导航栈，包括：
- 全局路径规划
- 局部路径规划
- 障碍物避障
- 动态重规划

## 多机器人 Namespace 支持

本系统支持通过 `ns` 参数实现多个导航系统的隔离，适用于多机器人场景。

### Topic 隔离

当设置 `ns:=rkbot` 时，话题自动加上 namespace 前缀：

| 话题类型 | 无 namespace | 有 namespace (rkbot) |
|----------|-------------|---------------------|
| scan | `/scan` | `/rkbot/scan` |
| map | `/map` | `/rkbot/map` |
| lio/odom | `/lio/odom` | `/rkbot/lio/odom` |
| cmd_vel | `/cmd_vel` | `/rkbot/cmd_vel` |

**注意**：TF 话题 (`/tf`, `/tf_static`) 保持全局共享，以支持多机器人之间的坐标变换互通。

### Frame 隔离

Frame ID 会根据 namespace 自动添加前缀：

| Frame | 无 namespace | 有 namespace (rkbot) |
|-------|-------------|---------------------|
| map | `map` | `rkbot/map` |
| odom | `odom` | `rkbot/odom` |
| base_footprint | `base_footprint` | `rkbot/base_footprint` |
| base_link | `base_link` | `rkbot/base_link` |

### 注意事项

1. **LIO 算法兼容性**：目前仅 Super-LIO 完成改造支持 namespace。其他 LIO 算法如需多机器人支持，需进行类似改造。
2. **TF 树共享**：所有机器人的 TF 发布到全局 `/tf`，确保 rviz 等工具可以可视化所有机器人的坐标变换。
3. **rviz 配置**：使用 namespace 时，rviz 中的 topic 和 frame 需要相应调整。

## 配置说明

### 全局配置

项目使用 `global_config` 包进行统一配置管理，支持环境变量配置：
- `MANUAL_BUILD_MAP`: 手动建图模式
- `AUTO_BUILD_MAP`: 自动建图模式
- `NAVIGATION_MODE`: 导航模式设置
- `SLAM_ALGORITHM`: SLAM算法选择

### base_footprint 配置

base_footprint 已集成到各 LIO 算法中，可在各算法的配置文件中设置：

```yaml
lio.output:
  footprint_pub_en: true
  tf_base_footprint_frame: "base_footprint"
```

## Web控制界面

系统启动后，可通过Web浏览器访问控制界面：
- 访问地址：`http://localhost:8083/nav2_web_control.html`
- 功能：地图显示、机器人位置标记、目标点设置、位置源切换

## 地图保存与转换

```bash
# 保存点云地图
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.05, destination: '/projects/LOAM/'}"

# 转换为占用网格地图
ros2 run nav2_map_server map_saver_cli -t /projected_map -f /path/to/map --fmt png
```

## 项目结构

```
LIO-SAM_MID360_ROS2_PKG/
├── ros2/src/
│   ├── LIO-SAM_MID360_ROS2_DOG/      # LIO-SAM实现
│   ├── FAST_LIO_ROS2_edit/           # FAST-LIO2实现
│   ├── point_lio_ros2/               # Point-LIO实现
│   ├── Super-LIO/                    # Super-LIO实现
│   ├── SC_PGO_ROS2/                  # SC-PGO姿态图优化
│   ├── nav2_dog_slam/                # 统一导航系统
│   ├── livox_ros_driver2/            # Livox雷达驱动
│   ├── livox_gazebo_garden/          # Gazebo Garden仿真环境
│   ├── global_config/                # 全局配置管理
│   └── m-explore/                    # 自主探索
├── map_sample/                       # 地图示例
└── build_ros2.sh                     # 构建脚本
```

## 最新更新

- **2026-04-09**: Gazebo7可正常运行，删除旧gazebo版本，添加Gazebo Garden仿真支持
- **2026-04-08**: 模拟小车已能发出点云，优化MPPI和AMCL参数，建图导航都可在无namespace情况使用
- **2026-04-07**: 修正namespace问题，目前无namespace可建图
- **2026-04-06**: Web控制界面添加路线和footprint显示
- **2026-02-11**: 去除fasterlio和dio（未使用），修正pointlio的footprint参考问题，微调superlio点云保存参数
- **2026-02-10**: 将base_footprint移至各lio算法输出
- **2026-02-09**: 修正footprint计算
- **2026-02-08**: 改进base_footprint计算
- **2026-02-07**: 参数微调，superlio疑似会影响odom使其倾斜，需调试
- **2026-02-06**: super lio导航可用
- **2026-02-05**: 添加Super-LIO支持到SLAM导航系统
- **2026-02-04**: superlio添加body cloud
- **2026-01-30**: 修正base_footprint相关问题，实现稳定高速导航
- **2026-01-27**: 导航算法集中到一起，统一启动文件

## 故障排除

```bash
# 查看TF树
ros2 run tf2_tools view_frames.py

# 查看话题列表
ros2 topic list

# 查看节点信息
ros2 node list
```

### 常见问题

1. **雷达连接问题**: 检查Livox-SDK2是否正确安装
2. **TF变换错误**: 确认base_footprint坐标系设置正确
3. **导航失败**: 检查地图质量和导航参数配置

## 后续规划

### SLAM 方向
- FAST-LIO2 稳定性优化
- slam_toolbox 2D 定位集成
- 建图扭曲校正（IMU/外参/时间同步）
- 大地图 + 动态地图处理
- 3D 重定位与导航

### 导航方向
- 导航直行优化（MPPI/TEB）
- 超时问题处理
- 行为树优化
- 动态速度调整

### 硬件/算力方向
- RK3588 大小核调度
- DDS/ROS2 通信优化
- 内存和 swap 优化
- 外部硬件定位（UWB/蓝牙）

### 系统稳定性
- TF 延迟/漂移监控
- 自动恢复策略
- 远程调试 / OTA 合规化
