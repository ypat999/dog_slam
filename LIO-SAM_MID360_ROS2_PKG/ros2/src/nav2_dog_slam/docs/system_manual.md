# nav2_dog_slam 系统说明书

## 1. 项目概述

`nav2_dog_slam` 是一个面向四足机器狗的 ROS2 自主导航包，将 LIO-SLAM（激光惯性里程计）与 Nav2 导航框架深度集成，实现建图、定位、路径规划、自主导航和 GPS 融合定位功能。系统运行在 RK3588 平台上，使用 Livox MID360 激光雷达。

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     nav2_dog_slam 系统                       │
│                                                             │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────────────┐ │
│  │ 传感器层  │  │   SLAM 后端    │  │     导航决策层        │ │
│  │          │  │               │  │                      │ │
│  │ MID360   │──│  Super-LIO    │──│  Nav2 导航栈          │ │
│  │ IMU      │  │  Fast-LIO     │  │  (MPPI+SMAC+BT)     │ │
│  │ GPS      │  │  LIO-SAM      │  │                      │ │
│  │          │  │  Point-LIO    │  │                      │ │
│  └──────────┘  └───────────────┘  └──────────────────────┘ │
│                                                             │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────────────┐ │
│  │ 建图工具  │  │   定位模块     │  │     交互界面          │ │
│  │          │  │               │  │                      │ │
│  │slam_toolbox│ │  AMCL         │  │  Web 控制界面         │ │
│  │octomap   │  │  GPS 融合(EKF) │  │  (rosbridge+HTTP)   │ │
│  └──────────┘  └───────────────┘  └──────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 支持的 SLAM 算法

| 算法 | 点云话题 | 里程计话题 | 说明 |
|------|---------|-----------|------|
| Super-LIO | `/lio/body/cloud` | `/lio/odom` | 默认算法，综合性能最优 |
| Fast-LIO | `/cloud_registered_body` | `/Odometry` | 轻量级，适合资源受限场景 |
| LIO-SAM | `/lio_sam/mapping/cloud_registered_raw` | `/lio_sam/mapping/odometry` | 带回环检测 |
| Point-LIO | `/cloud_registered_body` | `/Odometry` | 高鲁棒性 |

---

## 2. 目录结构

```
nav2_dog_slam/
├── CMakeLists.txt                  # 构建配置
├── package.xml                     # 包依赖声明
├── config/
│   ├── nav2_params.yaml            # Nav2 参数（实机）
│   ├── nav2_params_sim.yaml        # Nav2 参数（仿真）
│   ├── gps_ekf.yaml                # EKF 融合参数
│   ├── navsat_transform.yaml       # GPS 坐标转换参数
│   ├── navigate_to_pose_w_replanning_and_recovery.xml  # 导航行为树
│   ├── robot.urdf.xacro            # 机器人模型
│   ├── robot.urdf_tilt.xacro       # 机器人模型（雷达倾斜）
│   └── rviz2.rviz                  # RViz 可视化配置
├── launch/
│   ├── lio_nav2_unified.launch.py  # 主启动文件（统一入口）
│   ├── nav2_gps_fusion.launch.py   # GPS 融合导航启动
│   ├── navigation_launch.py        # Nav2 导航栈启动
│   └── gps_fusion.launch.py        # GPS 融合模块启动
├── scripts/
│   ├── publish_initial_pose.py     # 发布初始位姿
│   ├── surf_map_matching.py        # ORB 特征扫描-地图匹配
│   ├── dynamic_base_footprint.py   # 动态 TF 发布
│   ├── verify_gps_config.py        # GPS 配置验证
│   ├── test_gps_fusion.py          # GPS 融合测试
│   ├── gps_test_publisher.py       # GPS 数据模拟发布
│   └── verify_gps_fusion.py        # GPS 融合数据流验证
├── src/
│   ├── gps_preprocessor.py         # GPS 数据预处理/滤波节点
│   └── gps_simulator.py            # GPS 数据模拟器
├── web/
│   ├── run_web.sh                  # Web 服务启动脚本
│   ├── nav2_web_control.html       # Web 控制界面
│   └── libs/                       # 前端依赖库
│       ├── roslib.min.js
│       ├── ros2d.min.js
│       ├── easeljs.min.js
│       └── eventemitter2.min.js
└── docs/
    ├── system_manual.md            # 本说明书
    └── gps_fusion_guide.md         # GPS 融合指南
```

---

## 3. Systemd 服务

系统通过两个 systemd 服务实现开机自启动，均运行在 `ROS_DOMAIN_ID=27`。

### 3.1 服务总览

| 对比项 | `lio_sam_buildmap.service`（建图） | `lio_sam_nav2.service`（导航） |
|--------|----------------------------------|-------------------------------|
| 启动脚本 | `run_buildmap.sh super_lio` | `run_navigation.sh super_lio` |
| 核心变量 | `MANUAL_BUILD_MAP=True` | `MANUAL_BUILD_MAP=False` |
| 建图工具 | `slam_toolbox` | 不使用（加载已有地图） |
| SLAM 算法 | `super_lio` | `super_lio` |
| Restart 策略 | `always`（总是重启） | `on-failure`（仅失败重启） |
| 依赖服务 | `network.target` | `network.target` + `robot_control.service` |

### 3.2 启动节点对比

| 节点/组件 | 建图服务 | 导航服务 | 功能说明 |
|:----------|:-------:|:-------:|:---------|
| Super-LIO | ✅ | ✅ | 激光惯性里程计，输出点云和里程计 |
| pointcloud_to_laserscan | ✅ | ✅ | 3D 点云转 2D 激光扫描 |
| rosbridge_websocket | ✅ | ✅ | ROS-WebSocket 桥接（端口 9090） |
| Web HTTP 服务 | ✅ | ✅ | 网页控制界面（端口 8083） |
| slam_toolbox（建图模式） | ✅ | ❌ | 接收 /scan 实时生成栅格地图 |
| map_server | ❌ | ✅ | 加载已保存的地图文件 |
| AMCL | ❌ | ✅ | 自适应蒙特卡洛定位 |
| gps_preprocessor | ❌ | ✅ | GPS 数据质量过滤 |
| navsat_transform_node | ❌ | ✅ | GPS 经纬度 → map 坐标转换 |
| ekf_filter_node | ❌ | ✅ | EKF 多传感器融合 |
| Nav2 导航栈 | ✅ | ✅ | 路径规划 + 运动控制 + 行为树 |

### 3.3 服务管理命令

```bash
# 启动/停止建图服务
sudo systemctl start lio_sam_buildmap.service
sudo systemctl stop lio_sam_buildmap.service

# 启动/停止导航服务
sudo systemctl start lio_sam_nav2.service
sudo systemctl stop lio_sam_nav2.service

# 设置开机自启
sudo systemctl enable lio_sam_buildmap.service
sudo systemctl enable lio_sam_nav2.service

# 查看服务状态
sudo systemctl status lio_sam_buildmap.service
sudo systemctl status lio_sam_nav2.service

# 查看服务日志
journalctl -u lio_sam_buildmap.service -f
journalctl -u lio_sam_nav2.service -f
```

> **注意**：两个服务不应同时运行，建图完成后需停止建图服务，再启动导航服务。

---

## 4. 工作模式详解

### 4.1 建图模式

**用途**：遥控机器狗在未知环境中行走，实时构建 2D 栅格地图。

**数据流**：

```
MID360 雷达 + IMU
       │
   Super-LIO
       │
       ├── /lio/body/cloud ──→ pointcloud_to_laserscan ──→ /scan
       │                                                      │
       │                                              slam_toolbox
       │                                                      │
       └── /lio/odom                                    /map（实时栅格地图）
```

**操作步骤**：

1. 启动建图服务：`sudo systemctl start lio_sam_buildmap.service`
2. 打开 Web 界面：浏览器访问 `http://<机器狗IP>:8083`
3. 遥控机器狗在目标区域行走，观察地图逐步生成
4. 建图完成后，保存地图：
   ```bash
   ros2 run nav2_map_server map_saver_cli -f /home/ztl/slam_data/grid_map/map
   ```
5. 停止建图服务：`sudo systemctl stop lio_sam_buildmap.service`

### 4.2 导航模式

**用途**：在已有地图上实现自主定位和导航。

**数据流**：

```
MID360 雷达 + IMU                         GPS 模块
       │                                    │
   Super-LIO                         gps_preprocessor
       │                                    │
       ├── /scan ──→ AMCL（定位）     navsat_transform
       │              │                     │
       │         map_server            ekf_filter
       │       （加载已有地图）               │
       │              │              /odometry/gps_fused
       │              │
       └──────── Nav2 导航栈
                      │
                 路径规划(SMAC) → 运动控制(MPPI) → /cmd_vel
```

**操作步骤**：

1. 确保已有地图文件：`/home/ztl/slam_data/grid_map/map.yaml`
2. 启动导航服务：`sudo systemctl start lio_sam_nav2.service`
3. 打开 Web 界面：浏览器访问 `http://<机器狗IP>:8083`
4. 在 Web 界面上点击设置初始位姿（如果 AMCL 定位不准确）
5. 在 Web 界面上点击设置目标点，机器狗将自动规划路径并导航

---

## 5. 核心组件配置

### 5.1 Nav2 导航栈参数（nav2_params.yaml）

| 组件 | 配置项 | 值 | 说明 |
|------|--------|-----|------|
| **控制器** | 算法 | MPPI | 模型预测路径积分，适合复杂地形 |
| **规划器** | 算法 | SMAC Hybrid-A* | 支持非完整约束路径规划 |
| **行为树** | XML | navigate_to_pose_w_replanning_and_recovery.xml | 1Hz 重规划 + 多种恢复行为 |
| **全局代价地图** | 分辨率 | 0.05m | 使用 voxel_layer 体素图层 |
| **局部代价地图** | 范围 | 由参数文件定义 | 使用 voxel_layer 体素图层 |
| **AMCL** | 粒子数 | 由参数文件定义 | 自适应蒙特卡洛定位 |
| **速度平滑器** | 输出话题 | /cmd_vel | 对 /cmd_vel_nav 进行平滑 |

### 5.2 行为树恢复策略

导航过程中遇到障碍时，系统按以下顺序执行恢复行为（最多重试 3 次）：

| 顺序 | 恢复行为 | 参数 | 说明 |
|------|---------|------|------|
| 1 | 清除代价地图 | 全局 + 局部 | 清除可能的虚假障碍 |
| 2 | 后退 | 0.5m, 0.1m/s | 退出卡住位置 |
| 3 | 等待 | 2 秒 | 等待动态障碍物移开 |
| 4 | 旋转 | 90°(1.57rad) | 尝试找到新的可通行方向 |

### 5.3 GPS 融合参数

| 参数文件 | 关键配置 | 说明 |
|---------|---------|------|
| `gps_ekf.yaml` | 融合 /Odometry + /gps/fix_filtered + /imu/data | EKF 在 map 坐标系下融合 |
| `navsat_transform.yaml` | world_frame: map, base_link_frame: base_link | GPS 经纬度转 map 坐标 |

**GPS 预处理滤波条件**：

| 条件 | 阈值 | 说明 |
|------|------|------|
| 定位状态 | ≥ 0 (STATUS_FIX) | 丢弃无效定位 |
| 卫星数 | ≥ 4 | 至少 4 颗卫星 |
| HDOP | ≤ 2.0 | 水平精度因子 |
| NaN 检测 | 过滤 | 丢弃无效坐标 |

### 5.4 CPU 核心绑定

系统对关键节点进行了 CPU 核心绑定以优化 RK3588 性能：

| 节点 | CPU 核心 | 说明 |
|------|---------|------|
| pointcloud_to_laserscan | CPU 5 | 独占核心，保证实时性 |
| slam_toolbox | CPU 5, 6 | 建图计算量较大 |
| map_server | CPU 0-3 | 地图加载 |
| AMCL | CPU 5, 6 | 定位计算 |

---

## 6. ROS2 话题与坐标系

### 6.1 关键话题

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/lio/body/cloud` | PointCloud2 | Super-LIO → | 机体坐标系下的点云 |
| `/lio/odom` | Odometry | Super-LIO → | 里程计 |
| `/scan` | LaserScan | → AMCL/slam_toolbox | 2D 激光扫描 |
| `/map` | OccupancyGrid | map_server/slam_toolbox → | 栅格地图 |
| `/fix` | NavSatFix | GPS → | 原始 GPS 数据 |
| `/fix_filtered` | NavSatFix | gps_preprocessor → | 过滤后的 GPS |
| `/odometry/gps_fused` | Odometry | ekf_filter → | GPS 融合里程计 |
| `/amcl_pose` | PoseWithCovarianceStamped | AMCL → | AMCL 定位位姿 |
| `/initialpose` | PoseWithCovarianceStamped | → AMCL | 初始位姿设置 |
| `/goal_pose` | PoseStamped | → Nav2 | 导航目标点 |
| `/cmd_vel` | Twist | Nav2 → 底盘 | 速度指令 |

### 6.2 TF 坐标系树

```
       map
        │
       odom
        │
   base_footprint
        │
     base_link
      ├── livox_frame   (激光雷达)
      └── imu_link      (IMU)
```

| 变换 | 发布者 | 说明 |
|------|--------|------|
| map → odom | AMCL / slam_toolbox | 定位校正 |
| odom → base_footprint | Super-LIO (C++) | 里程计 |
| base_footprint → base_link | URDF (static) | 固定变换 |
| base_link → livox_frame | URDF (static) | 雷达安装位置 |
| base_link → imu_link | URDF (static) | IMU 安装位置 |

---

## 7. Web 控制界面

### 7.1 访问方式

- **地址**：`http://<机器狗IP>:8083/nav2_web_control.html`
- **依赖**：rosbridge_websocket 运行在端口 9090

### 7.2 功能列表

| 功能 | 操作方式 | 说明 |
|------|---------|------|
| 地图显示 | 自动 | 实时显示 /map 栅格地图 |
| 缩放/平移 | 鼠标滚轮/拖拽 | 地图视角控制 |
| 机器人位置 | 自动 | 显示 AMCL/slam_toolbox/LIO 位姿 |
| 设置初始位姿 | 点击+拖拽 | 发布 /initialpose |
| 设置导航目标 | 点击+拖拽 | 发布 /goal_pose |
| 取消导航 | 按钮 | 停止当前导航任务 |

---

## 8. 全局配置（global_config）

系统通过 `global_config` 包管理全局参数，支持环境变量覆盖和多主机适配。

### 8.1 环境变量

| 环境变量 | 默认值 | 说明 |
|---------|--------|------|
| `MANUAL_BUILD_MAP` | `False` | `True` 为建图模式，`False` 为导航模式 |
| `BUILD_TOOL` | `octomap_server` | 建图工具：`slam_toolbox` 或 `octomap_server` |
| `SLAM_ALGORITHM` | `fast_lio` | SLAM 算法选择 |
| `AUTO_BUILD_MAP` | `False` | 自动探索建图模式 |
| `NAVIGATION_MODE` | `standalone` | 导航模式 |
| `ROS_DOMAIN_ID` | `27` | ROS2 域 ID |

### 8.2 多主机配置

系统根据 `hostname` 自动选择配置：

| 主机名 | 用途 | use_sim_time | ONLINE_LIDAR |
|--------|------|:------------:|:------------:|
| `RK3588` | 实机（机器狗） | `false` | `true` |
| `jqr001` | 开发机 | `true` | `false` |
| `DESKTOP-4LS1SSN` | 仿真/开发 | `true` | `false` |
| `DESKTOP-ypat` | 仿真/开发 | `true` | `false` |

---

## 9. 手动启动命令

除了 systemd 服务外，也可以手动启动。

### 9.1 建图模式

```bash
# 使用 Super-LIO + slam_toolbox 建图
cd /home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG
./scripts/run_buildmap.sh super_lio slam_toolbox

# 使用 Fast-LIO + octomap 建图
./scripts/run_buildmap.sh fast_lio octomap_server
```

### 9.2 导航模式

```bash
# 使用 Super-LIO 导航
cd /home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG
./scripts/run_navigation.sh super_lio

# 使用 Fast-LIO 导航
./scripts/run_navigation.sh fast_lio
```

### 9.3 停止所有节点

```bash
./scripts/stop_all_navs.sh
```

---

## 10. 常用调试命令

```bash
# 查看当前运行的节点
ros2 node list

# 查看话题列表
ros2 topic list

# 查看 TF 树
ros2 run tf2_tools view_frames

# 监听里程计
ros2 topic echo /lio/odom

# 监听激光扫描
ros2 topic hz /scan

# 监听地图发布频率
ros2 topic hz /map

# 监听 GPS 数据
ros2 topic echo /fix

# 监听融合后 GPS 里程计
ros2 topic echo /odometry/gps_fused

# 监听导航速度指令
ros2 topic echo /cmd_vel

# 查看 AMCL 定位
ros2 topic echo /amcl_pose

# 保存地图（建图完成后）
ros2 run nav2_map_server map_saver_cli -f /home/ztl/slam_data/grid_map/map

# 发布初始位姿
ros2 run nav2_dog_slam publish_initial_pose.py

# GPS 融合测试
ros2 run nav2_dog_slam test_gps_fusion.py

# GPS 配置验证
ros2 run nav2_dog_slam verify_gps_config.py
```

---

## 11. 故障排除

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 服务启动失败 | ROS2 环境未正确安装 | 检查 `source /opt/ros/humble/setup.bash` |
| 无法看到地图 | 建图模式下 slam_toolbox 未收到 /scan | 检查 Super-LIO 是否正常输出点云 |
| AMCL 定位漂移 | 初始位姿不准确 | 通过 Web 界面重新设置初始位姿 |
| 导航不动 | 代价地图有虚假障碍 | 清除代价地图或检查传感器数据 |
| Web 界面无法访问 | HTTP 服务或 rosbridge 未启动 | 检查 8083 和 9090 端口 |
| GPS 融合无输出 | GPS 数据被预处理滤波丢弃 | 降低 `min_satellites` 或增大 `max_hdop` |
| 导航路径抖动 | MPPI 参数不合适 | 调整 nav2_params.yaml 中控制器参数 |
| 服务频繁重启 | 底层节点崩溃 | `journalctl -u <service> -f` 查看日志 |

---

## 12. 依赖包列表

| 类别 | 包名 | 用途 |
|------|------|------|
| **SLAM** | `super_lio` / `fast_lio` / `lio_sam` / `point_lio` | 激光惯性里程计 |
| **点云处理** | `pointcloud_to_laserscan` | 3D → 2D |
| **建图** | `slam_toolbox` / `octomap_server` | 栅格地图生成 |
| **定位** | `nav2_amcl` | AMCL 定位 |
| **导航** | `nav2_controller` / `nav2_planner` / `nav2_bt_navigator` | Nav2 核心 |
| **导航辅助** | `nav2_behaviors` / `nav2_smoother` / `nav2_velocity_smoother` | 行为恢复/平滑 |
| **地图** | `nav2_map_server` | 地图加载/保存 |
| **生命周期** | `nav2_lifecycle_manager` | 节点生命周期管理 |
| **GPS** | `robot_localization` | EKF 融合 + navsat 变换 |
| **Web** | `rosbridge_server` | WebSocket 桥接 |
| **模型** | `robot_state_publisher` / `joint_state_publisher` / `xacro` | URDF 发布 |
| **配置** | `global_config` | 全局参数管理 |
