# FAST-LIO2 & LIO-SAM MID360 ROS2 Package

基于ROS2 Humble的Livox MID360激光雷达SLAM与导航系统，集成了多种先进的SLAM算法和导航功能。

## 主要特性

- **多SLAM算法支持**: FAST-LIO2, LIO-SAM, Point-LIO, Super-LIO
- **统一导航系统**: 集成Nav2导航框架，支持自主探索和路径规划
- **多机器人支持**: 通过 namespace 实现多个导航系统隔离（TF已支持namespace）
- **实时建图**: 支持在线建图和离线地图构建
- **稳定导航**: 优化base_footprint，实现高速稳定导航
- **可通行区域检测**: 纯三维可通行区域检测Nav2 costmap插件，支持楼梯检测
- **中狗适配**: 支持中狗（中型机器人）Livox MID360双雷达配置
- **多Planner配置**: 支持多种全局/局部路径规划器切换
- **Gazebo Garden仿真**: 支持Gazebo Garden GPU加速仿真，提供多种仿真环境

## 技术优势

相对于市面上的开源 / 商业导航系统，本系统在二维和三维两个层面具有以下差异化技术优势。

### 一、三维导航层面（核心差异化）

#### 1. OctoPlanner3D —— 真正的三维可通行性规划
市面 3D 导航普遍只是「2D costmap + Octomap 障碍层」，仍在平面内规划。本系统集成基于 OctoMap 体素的 A\* 三维路径搜索，支持楼梯、坡地等高程变化路径：

- A\* 搜索中引入**方向一致性 + 对角线跨越惩罚**，减少阶跃折角
- **两阶段梯度下降平滑**：先 Laplace 平滑，再 cost 微调，z 梯度禁用避免路径下沉
- **cost 场补全 + 梯度优化**替代逐次碰撞检查，规划时间显著下降
- 占据缓存改逐格查询 + 粗叶子展开 + 重层扫描提速
- A\* 失败时清空 `planner_results_`，杜绝 stale path
- 终点方向修正 + 路径时效性 bug 修复

#### 2. Nav2 风格指数衰减 cost 场 —— 大膨胀半径下仍可穿越窄通道
- 指数衰减 cost 场支持大膨胀半径 + 窄通道可规划（市面 costmap_common_params 的痛点）
- `preblocked` 列扫描范围缩小至 `support_depth`，修复 2m 窄通道规划失败
- cost intensity 可视化 + 蓝色路径测试模式（不影响真实导航）

#### 3. 纯三维可通行性 costmap 插件（traversability_layer）
非普通障碍层，而是基于三维点云的：

- 地形坡度检测：识别不可通行陡坡
- 楼梯检测：支持机器人楼梯攀爬导航（市面 Nav2 插件几乎没有）
- 三维通行性分析：障碍物高度、密度综合判断
- **永久 cost 记忆 + 信任插值地面 + 次数制衰减**：对动态遮挡、雷达盲区更鲁棒
- costmap origin 驱动的滚动窗口 + backbuffer 双缓冲 + mutex 分离，多线程安全
- 跳帧机制降低 CPU 占用

#### 4. 3D 专用行为树 + Action 适配器无缝集成 Nav2
- 3D 专用 BT XML（移除 global_costmap 引用）
- ComputePathThroughPoses action + 时序修复
- 2D / 3D 导航共用 Nav2 行为树、生命周期、恢复行为，无需替换整个导航栈

### 二、二维导航层面

#### 1. 多 SLAM 算法统一热切换
通过 `lio_nav2_unified.launch.py` 统一入口 + `SLAM_ALGORITHM` 环境变量，在 **FAST-LIO2 / Point-LIO / Super-LIO / LIO-SAM** 之间无感切换，且都基于 Livox MID360 三维雷达，相比传统 2D 激光 SLAM 信息密度高一个量级。

#### 2. 原生多机器人 Namespace 隔离（Topic / Frame / TF 三层）
商业多机器人方案常依赖独立 master 或 bridge。本系统做到：

- Topic namespace 隔离（`/scan` → `/rkbot/scan`）
- Frame namespace 隔离（`map` → `rkbot/map`）
- TF 话题 namespace 隔离，规避多机 TF 冲突
- 同机即可启动多套导航实例，开源 Nav2 默认不提供

#### 3. SC-PGO 姿态图优化
商业 2D 导航几乎不做姿态图优化，本系统借鉴 SC-LIO 思路：

- **双重距离判断回环检测**（原始 odom + PGO 优化后），降低误闭环
- 建图流程加入**动态物体去除**
- GICP 候选限 5 个，保证实时性
- 在线 / 离线双模式 + 多线程稳定性

#### 4. 定位退化方向检测（lidar_localization_ros2）
传统 NDT/ICP 定位只看总 fitness，本系统可发现「总分明但单轴漂」的退化场景：

- **per-axis fitness score 阈值检查**，识别走廊 / 长直走廊等退化方向
- per-axis 归一化，量纲与 PCL `getFitnessScore()` 一致
- `min_localization_interval` 防止定位失败后 CPU 空转
- 用配置阈值替代 `DBL_MAX` 硬编码，异常值（如 fitness=685110）正确拒绝
- 定位失败时更新 odom 参考点，避免 CPU 空转死循环

### 三、整体系统层面优势

| 维度 | 市面典型方案 | 本系统 |
|------|------------|--------|
| 雷达 | 2D LiDAR 或机械式 3D | Livox MID360 非重复扫描，性价比高 |
| 多机部署 | 商业多机调度软件 | ns 原生隔离，开源可复现 |
| SLAM | 单一算法 | 4 算法统一接口热切换 |
| 闭环优化 | 无或简单 | SC-PGO 双重距离判断 + 动态物去除 |
| 定位退化 | 无检测 | per-axis fitness 退化方向检测 |
| 3D 路径规划 | 2D + 高程 | 真 3D A\* + 楼梯/坡度通行性 |
| costmap | Obstacle/Voxel | 永久记忆 + 信任插值 + 三维通行性 |
| 行为树 | 默认 BT | 2D / 3D 专用 BT XML |
| 仿真 | Gazebo Classic | Gazebo Garden GPU 加速 |
| Web 控制 | 闭源 HMI | rosbridge + HTML 控制台 |
| 硬件适配 | 通用 | 中狗双雷达 / RK3588 大小核 / Orin 多主机配置 |

### 四、核心差异化优势总结

1. **三维可通行性真 3D 规划**（OctoPlanner3D + traversability_layer）：可在楼梯、坡地等高程变化环境做路径规划，市面开源 Nav2 几乎没有，商业方案多为闭源。
2. **定位退化方向检测 + SC-PGO 双重闭环**：长走廊、规则厂房、大场景建图导航的鲁棒性显著优于纯 LIO + AMCL 方案。
3. **多 SLAM 算法统一接口 + 多机 namespace 原生支持**：同一套代码覆盖从研究到多机部署，避免商业方案的算法锁定。

## 系统要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS2 Humble
- **硬件**: Livox MID360激光雷达
- **依赖库**: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

### 推荐使用阿里源：
```bash
deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu/ jammy main
deb https://mirrors.aliyun.com/ubuntu/ jammy main

# 添加gpg key
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### TinyProxy 网络代理配置

为 192.168.168.100 提供网络代理服务：

```bash
# 安装 tinyproxy
sudo apt update
sudo apt install -y tinyproxy

# 配置 tinyproxy 允许访问的客户端 IP 和端口
sudo sed -i 's/^Port 8888/Port 7890/' /etc/tinyproxy/tinyproxy.conf
sudo sed -i '/^Allow 127.0.0.1$/a Allow 10.0.0.0/8\nAllow 192.168.0.0/16' /etc/tinyproxy/tinyproxy.conf

# 重启 tinyproxy 服务
sudo systemctl restart tinyproxy
sudo systemctl enable tinyproxy

# 验证服务状态
sudo systemctl status tinyproxy

# 在客户端上使用代理
export http_proxy=http://192.168.168.151:7889
export https_proxy=http://192.168.168.151:7889

# 配置 apt 代理
sudo tee /etc/apt/apt.conf.d/proxy.conf << 'EOF'
Acquire::http::Proxy "http://192.168.168.151:7889";
Acquire::https::Proxy "http://192.168.168.151:7889";
EOF

# 链式代理
# conf添加：Upstream http 10.0.4.185:10811
# 服务端前台启动
sudo tinyproxy -d -c /etc/tinyproxy/tinyproxy-chain.conf

# 内网机git代理配置
git config --global http.proxy "http://192.168.168.151:7890"
git config --global https.proxy "http://192.168.168.151:7890"
git config --global http.sslVerify false

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
# GTSAM (LIO-SAM依赖)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# SLAM相关依赖
sudo apt install -y ros-humble-perception-pcl ros-humble-pcl-msgs \
    ros-humble-vision-opencv ros-humble-xacro ros-humble-vision-msgs

# Super-LIO依赖
sudo apt install libgoogle-glog-dev libtbb-dev

# Nav2导航系统
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-rosbridge-server 
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
    ros-humble-rosbridge-server \
    ros-humble-spatio-temporal-voxel-layer libopenvdb-dev

# 点云转激光扫描
sudo apt install -y ros-humble-pointcloud-to-laserscan

# OctoMap建图
sudo apt install -y ros-humble-octomap ros-humble-octomap-msgs
sudo apt install -y ros-humble-octomap-server

# rqt
sudo apt install -y ros-humble-rqt  

# slam_toolbox
sudo apt install -y ros-humble-slam-toolbox

```





# Gazebo Garden 仿真环境
```bash
sudo apt update
sudo apt install -y lsb-release curl gnupg

# 添加 GZ Sim GPG key
curl -sSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /etc/apt/trusted.gpg.d/gazebo.gpg > /dev/null

# 添加 Garden 软件源（Ubuntu 22.04 用 jammy）
echo "deb http://packages.osrfoundation.org/gz-garden/ubuntu `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/gz-garden.list > /dev/null

sudo apt update
sudo apt install -y mesa-vulkan-drivers vulkan-tools

# 安装 GZ Garden
sudo apt install -y gz-garden

# 安装 ROS2 与 Gazebo Garden 桥接及相关控制包
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros2-control* \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro

# Python 依赖（launch 文件运行需要）
pip install PyYAML lark packaging
```

#############################################
# 若Gazebo缺少基础模型，可手动克隆官方模型库
#############################################
```bash
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models

sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*
```

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
cd

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
- 全局路径规划：支持Navfn、SMAC等多种规划器
- 局部路径规划：支持MPPI、DWB、TEB等多种规划器
- 障碍物避障
- 动态重规划

### 可通行区域检测（三维）

系统集成了纯三维可通行区域检测插件 `traversability_layer`，基于点云数据进行实时地形分析：

- **地形坡度检测**：分析地面坡度，识别不可通行的陡坡区域
- **楼梯检测**：识别楼梯结构，支持机器人楼梯攀爬导航
- **动态跟随**：可通行区域检测随机器人移动实时更新
- **三维通行性分析**：基于三维点云的障碍物高度、密度分析

作为Nav2 costmap插件，可在导航配置中启用：

```yaml
local_costmap:
  plugins:
    - {name: traversability, type: "traversability_layer::TraversabilityLayer"}
```

### Planner多配置

系统支持多种全局/局部路径规划器配置，可通过参数切换：

```bash
# 使用不同Planner组合（在launch文件中配置）
# 全局规划器: Navfn / SMAC Lattice / A*
# 局部规划器: MPPI / DWB / TEB
```

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

**注意**：TF 话题 (`/tf`, `/tf_static`) 现已支持 namespace 隔离，可通过配置切换为全局共享或 namespace 隔离模式。

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
2. **TF 树隔离**：TF 话题现已支持 namespace 隔离，每个机器人的 TF 发布在各自的 namespace 下，避免多机器人 TF 冲突。可通过配置切换为全局共享模式。
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
│   ├── OctoPlanner3D-ROS2/           # 三维可通行性规划器（符号链接）
│   ├── traversability_layer/         # 纯三维可通行区域检测Nav2 costmap插件
│   ├── lidar_localization_ros2/      # 激光雷达定位
│   ├── ndt_omp_ros2/                 # NDT OMP匹配定位
│   ├── zsi_tools/                    # ZSI工具（中狗双雷达、Fast-LIO Airy等）
│   ├── autorccar_interfaces/         # 自动遥控车接口定义
│   ├── livox_ros_driver2/            # Livox雷达驱动
│   ├── livox_gazebo_garden/          # Gazebo Garden仿真环境
│   ├── global_config/                # 全局配置管理（OctoPlanner参数已集成）
│   └── m-explore/                    # 自主探索
├── map_sample/                       # 地图示例
└── build_ros2.sh                     # 构建脚本
```

### 三维导航系统（OctoPlanner3D）

系统集成了独立的 [OctoPlanner3D-ROS2](https://github.com/ypat999/OctoPlanner3D-ROS2) 子模块（原为symlink），提供基于OctoMap的三维可通行性导航能力：

- **三维可通行性分析**：基于OctoMap体素化分析三维地形可通行性
- **智能路径规划**：A*算法搜索三维可通行路径（支持方向一致性/对角线跨越惩罚），避免楼梯、陡坡等不可通行区域
- **路径平滑**：两阶段梯度下降（纯Laplace平滑 + cost微调），cost场补全优化
- **占据缓存加速**：逐格查询、粗叶子展开、重层扫描提速
- **Nav2深度集成**：通过`OctoPlannerGlobalPlanner`插件直接作为planner_server后端，移除独立adapter
- **3D专用行为树**：独立BT XML（`nav2_bt_navigator_3d.xml`），移除global_costmap引用
- **独立参数文件**：`nav2_params_3d.yaml`统一管理所有OctoPlanner和3D导航参数
- **3D Web控制界面**：支持三维地图显示和导航控制
- **双模式运行**：
  - **导航模式**：`initialpose` + `goal_pose`（Nav2执行）
  - **测试模式**：`clicked_point`（路径可视化，不影响导航）
- **配置统一管理**：input_pcd路径由`global_config`自动适配不同主机
- **Namespace支持**：支持多机器人场景的namespace隔离

#### 3D导航启动方式

```bash
# 启动3D导航系统（OctoPlanner + Nav2）
MANUAL_BUILD_MAP=False SLAM_ALGORITHM=super_lio ros2 launch nav2_dog_slam lio_nav2_unified_3d.launch.py

# 使用其他SLAM算法
MANUAL_BUILD_MAP=False SLAM_ALGORITHM=fast_lio ros2 launch nav2_dog_slam lio_nav2_unified_3d.launch.py
MANUAL_BUILD_MAP=False SLAM_ALGORITHM=lio_sam ros2 launch nav2_dog_slam lio_nav2_unified_3d.launch.py
```

#### 3D导航配置

配置文件位于 `OctoPlanner3D-ROS2/config/params.yaml`，通过 `global_config` 自动适配不同主机：

```yaml
# 不同主机配置示例（在global_config/__init__.py）
'ywj-B250-D3A': {
    'OCTOPLANNER_PCD_FILE_PATH': '/home/ywj/slam_data/pcd/octomap.pcd',
    'OCTOPLANNER_PARAMS_FILE': '/path/to/OctoPlanner3D-ROS2/config/params.yaml',
}
```

#### 3D导航测试

```bash
# 发送3D导航目标（带朝向）
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 0.3}, orientation: {w: 1.0}}}"

# 测试模式：RViz Publish Point工具（蓝色路径，不影响导航）
# 第1次点击：起点
# 第2次点击：终点（自动规划测试路径）
```

## 最新更新

- **2026-07-15**: OctoPlanner3D独立子模块化（原symlink），A*方向一致性/对角线惩罚，两阶段梯度下降路径平滑，占据缓存优化提速；激光定位双阈值（初始0.5/持续0.02）；Super-LIO时间戳统一修复PGO错位、fitness score优化；SC-PGO小狗参数优化；3D专用BT XML和nav2_params_3d.yaml；3D Web控制界面；OctoPlanner配置统一到global_config；pointcloud_to_laserscan namespace修复
- **2026-07-27**: README 新增「技术优势」章节，突出 2D/3D 双层导航差异化能力；traversability_layer 优化 TF 漂移、backbuffer 双缓冲、mutex 分离、Z 轴收缩
- **2026-07-08**: 集成OctoPlanner3D三维导航系统，支持三维可通行性规划和Nav2集成
- **2026-05-27**: 可通行检测优化随机器人移动；TF加入namespace支持多机器人隔离
- **2026-05-26**: 适配中狗Livox MID360雷达配置；导航使用本体雷达；多配置2种全局/局部planner
- **2026-05-25**: Web控制界面修正与调整
- **2026-05-23**: 可通行区域检测重写为纯三维Nav2 costmap插件（traversability_layer）
- **2026-05-22**: 中狗本体点云服务自动启停；前后雷达不跳帧优化；修正Web与loop closure
- **2026-05-21**: 楼梯检测效果初步良好；前后雷达降采样率分配调整
- **2026-05-20**: 可通行性检测与坡度阈值调整，纯三维通行检测基本可用
- **2026-05-19**: 可通行性检测层开发（纯三维版本）
- **2026-05-16**: 增加仿真world环境
- **2026-05-13**: 中狗本体雷达服务适配，服务防停止卡死
- **2026-05-10**: 修正若干问题，小狗稳定导航
- **2026-05-09**: 仿真小车添加前后雷达
- **2026-05-06**: 添加中狗service、TinyProxy步骤、中狗雷达融合、更新readme
- **2026-04-27**: 添加orin配置
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
4. **OctoPlanner PCD文件加载失败**:
   ```bash
   # 检查global_config配置
   python3 -c "from global_config.global_config import OCTOPLANNER_PCD_FILE_PATH; print(OCTOPLANNER_PCD_FILE_PATH)"
   # 确认PCD文件存在
   ls -l $OCTOPLANNER_PCD_FILE_PATH
   ```
5. **OctoPlanner规划失败**:
   ```bash
   # 查看日志
   ros2 topic echo /rosout | grep octo_planner
   # 检查OctoMap缓存状态
   ls -l /path/to/octomap.bt*
   ```
6. **Nav2与OctoPlanner集成失败**:
   ```bash
   # 检查Action接口
   ros2 action list | grep compute_path_to_pose
   # 检查planner_server状态
   ros2 node list | grep planner_server
   # 检查octo_planner节点
   ros2 node list | grep octo_planner
   ```

## 后续规划

### SLAM 方向
- FAST-LIO2 稳定性优化
- slam_toolbox 2D 定位集成
- 建图扭曲校正（IMU/外参/时间同步）
- 大地图 + 动态地图处理
- 3D 重定位与导航
- Loop closure 优化

### 导航方向
- **OctoPlanner3D 路径平滑优化**（已完成）：两阶段梯度下降、cost场补全
- **OctoPlanner3D 性能优化**（已完成）：占据缓存加速、粗叶子展开、重层扫描提速
- **3D 导航集成优化**（已完成）：独立子模块、3D BT XML、nav2_params_3d.yaml、3D Web界面
- **激光定位增强**（已完成）：双阈值定位（初始宽松/持续严格）、全帧质量检查
- **OctoPlanner3D 稳定性验证**：集成测试、异常处理、实际环境验证
- **OctoPlanner3D 参数调优**：robot_radius、voxel分辨率、可通行性阈值、A*启发函数权重
- **3D vs 2D对比验证**：坡地、楼梯、复杂环境对比测试
- 可通行区域检测完善（楼梯检测、坡度分析）
- 导航直行优化（MPPI/TEB）
- 超时问题处理
- 行为树优化
- 动态速度调整

### 硬件/算力方向
- 中狗适配完善（双雷达融合、本体点云服务）
- RK3588 大小核调度
- DDS/ROS2 通信优化
- 内存和 swap 优化
- 外部硬件定位（UWB/蓝牙）

### 系统稳定性
- TF 延迟/漂移监控
- 自动恢复策略
- 前后雷达同步优化
- 远程调试 / OTA 合规化
