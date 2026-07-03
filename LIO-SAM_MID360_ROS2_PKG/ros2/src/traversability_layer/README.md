# Traversability Layer - 3D可通行性代价地图层

## 概述

Traversability Layer 是一个 nav2_costmap_2d 插件，用于从3D点云数据生成可通行性代价地图。相比传统2.5D高程图方案，它解决了室内环境中天花板被误判为不可通行区域的问题。

### 解决的核心问题

1. **天花板误判**：传统2.5D高程图取每个cell的最高点作为地面高度，室内环境中天花板会被当作不可通行区域
2. **平行墙体盲区**：与视线平行的墙体在raycast找梯度时可能找到相邻等高墙体，导致无cost

## 核心算法流程

```
点云输入 (PointCloud2)
    ↓
[1] 3D Voxel Grid 构建 (OpenMP并行)
    ↓
[2] Ray Tracing: 从传感器原点投射射线
    ↓ 区分地面点(hit + 上方free)和天花板点(hit + 下方free)
[3] 地面提取 (Ground Extraction)
    ↓ 计算每个cell的地面高度z
[4] 障碍物比率检测 (Obstacle Ratio)
    ↓ 检测地面上方robot_height范围内的障碍密度
    ↓ 解决平行墙体盲区问题
[5] 立方插值补全 (Interpolation)
    ↓ 填充无观测数据的地面cell
[6] 坡度与高度差计算
    ↓ 计算slope_x, slope_y, slope_magnitude, height_diff
[7] 代价计算 → Costmap2D
    LETHAL: 坡度>阈值 或 障碍比率>阈值
    INFLATED: 坡度/高度差线性加权
    FREE: 可通行区域
```

## 关键数据结构

### VoxelData - 3D体素
```cpp
struct VoxelData
{
  uint8_t hit_count = 0;   // 点云命中计数
  uint8_t pass_count = 0;  // 射线穿过计数(free space)
};
```

### GroundCell - 地面cell
```cpp
struct GroundCell
{
  float ground_z = 0.0f;          // 地面高度
  bool has_ground = false;        // 是否有地面数据
  float height_diff = 0.0f;       // 邻域最大高度差
  float slope_x = 0.0f;          // X方向坡度
  float slope_y = 0.0f;          // Y方向坡度
  float slope_magnitude = 0.0f;  // 坡度幅值
  float obstacle_ratio = 0.0f;   // 障碍物比率
};
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pointcloud_topic` | `/lio/body/cloud` | 点云输入话题 |
| `sensor_frame` | `""` (自动) | 传感器坐标系 |
| `voxel_z_resolution` | 0.1m | 体素Z轴分辨率 |
| `voxel_z_min` | -1.0m | 体素Z轴最小值 |
| `voxel_z_max` | 3.0m | 体素Z轴最大值 |
| `ground_hit_threshold` | 1 | 地面判定最小hit数 |
| `free_space_threshold` | 1 | 自由空间判定阈值 |
| `free_space_window` | 3 | 自由空间搜索窗口(Z方向cell数) |
| `robot_height` | 0.5m | 机器人高度（用于障碍比率检测） |
| `obstacle_ratio_threshold` | 0.5 | 障碍比率阈值（超过则LETHAL） |
| `max_slope_traversable` | 45.0° | 最大可通行坡度 |
| `slope_cost_start` | 15.0° | 开始产生cost的坡度 |
| `step_height_threshold` | 0.15m | 台阶高度阈值 |
| `height_cost_start` | 0.0m | 开始产生cost的高度差 |
| `slope_cost_scale` | 5.0 | 坡度cost缩放系数 |
| `height_cost_scale` | 10.0 | 高度差cost缩放系数 |
| `lethal_cost_threshold` | 254.0 | 致命cost阈值 |
| `observation_persistence` | 5.0s | 观测数据持久时间 |
| `cloud_buffer_size` | 5 | 点云缓冲帧数 |
| `interp_search_radius` | 3 | 插值搜索半径(cell数) |
| `min_interp_neighbors` | 2 | 插值最小邻居数 |
| `num_threads` | 0 (自动) | OpenMP线程数 |

## nav2 集成配置

在 nav2 参数文件中添加 traversability_layer 插件：

```yaml
local_costmap:
  ros__parameters:
    plugins: ["static_layer", "traversability_layer", "inflation_layer"]
    traversability_layer:
      plugin: "traversability_layer::TraversabilityLayer"
      enabled: true
      pointcloud_topic: "/lio/body/cloud"
      voxel_z_resolution: 0.1
      voxel_z_min: -1.0
      voxel_z_max: 3.0
      ground_hit_threshold: 1
      robot_height: 0.5
      obstacle_ratio_threshold: 0.5
      max_slope_traversable: 45.0
      step_height_threshold: 0.15
      num_threads: 0
```

## 性能优化

- **OpenMP并行**: 体素构建和地面提取均使用多线程，`num_threads=0`自动检测核心数
- **Thread-local缓冲**: 减少原子操作竞争，voxel写入使用线程局部缓冲后合并
- **局部costmap缓冲**: 消除cost写入的临界区
- **数据压缩**: voxel数据从uint16_t压缩为uint8_t，使用memset快速初始化

## 编译

```bash
cd ~/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select traversability_layer
```

## 备份

原始2.5D实现已备份为：
- `src/traversability_layer.cpp.bk`
- `include/traversability_layer/traversability_layer.hpp.bk`

## 项目位置

```
LIO-SAM_MID360_ROS2_PKG/ros2/src/traversability_layer/
├── include/traversability_layer/
│   ├── traversability_layer.hpp      # 头文件
│   └── traversability_layer.hpp.bk   # 原始2.5D版本备份
├── src/
│   ├── traversability_layer.cpp      # 3D实现
│   └── traversability_layer.cpp.bk   # 原始2.5D版本备份
├── traversability_layer_plugin.xml   # 插件描述
├── CMakeLists.txt
└── package.xml
```
