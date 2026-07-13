# OctoPlanner3D → Nav2 GlobalPlanner Plugin 重构计划

## 概述

将 OctoPlanner3D 从独立 ROS2 节点（自带 `ComputePathToPose` Action Server）改造为标准 Nav2 `GlobalPlanner` 插件，由 `planner_server` 通过 pluginlib 加载和管理。

## 当前架构 vs 目标架构

```
当前: bt_navigator → compute_path_to_pose action → octo_planner_rviz_node(独立)
目标: bt_navigator → compute_path_to_pose action → planner_server → pluginlib → OctoPlanner plugin
```

## 涉及文件

| 操作 | 文件 | 说明 |
|------|------|------|
| 新建 | `planner/src/octo_planner_global_planner.cpp` | GlobalPlanner 插件实现 |
| 新建 | `planner/include/octo_planner_global_planner.hpp` | 插件头文件 |
| 新建 | `plugin.xml` | pluginlib 导出清单 |
| 修改 | `CMakeLists.txt` | 添加插件库编译 + pluginlib 依赖 |
| 修改 | `package.xml` | 添加 nav2_core/nav2_costmap_2d/pluginlib 依赖 |
| 修改 | `src/octo_planner_rviz_node.cpp` | 移除 Action Server，保留可视化/测试功能 |
| 修改 | `config/params.yaml` | 添加 planner plugin 相关参数 |
| 修改 | `nav2_dog_slam/config/nav2_params_3d.yaml` | 添加 planner_server 配置段 |
| 修改 | `nav2_dog_slam/launch/navigation_launch_3d.py` | 增加 planner_server 启动 |
| 修改 | `nav2_dog_slam/launch/lio_nav2_unified_3d.launch.py` | 增加 planner_server lifecycle 管理 |

---

## Step 1: 创建 GlobalPlanner 插件头文件

**文件:** `planner/include/octo_planner_global_planner.hpp`

继承 `nav2_core::GlobalPlanner` 接口，将原 `OctoPlannerRvizNode` 中的：
- `executeComputePath()` 里的规划逻辑 → `createPlan()`
- 构造函数的参数读取 → `configure()`

```cpp
#pragma once

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "global_planner.h"
#include <memory>
#include <string>

namespace octo_planner3d
{

class OctoPlannerGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string logger_name_;
  bool is_configured_ = false;

  // 参数（从 params.yaml / nav2 node params 读取）
  double robot_radius_ = 0.25;
  int max_iterations_ = 800000;
  int snap_search_radius_cells_ = 12;
  bool require_ground_support_ = true;
  bool strict_direct_ground_support_ = false;
  int ground_support_xy_radius_cells_ = 1;
  int ground_support_depth_cells_ = 1;
  bool enable_preblocked_costmap_ = true;
  int preblocked_costmap_radius_cells_ = 3;
  double preblocked_costmap_weight_ = 2.5;
  bool lowest_traversable_only_ = false;

  // 平滑参数
  bool smoothing_enabled_ = true;
  double smoothing_simplify_epsilon_ = 0.1;
  double smoothing_interp_spacing_ = 0.15;
  int smoothing_gradient_iterations_ = 50;
  double smoothing_gradient_alpha_ = 0.3;

  int openmp_num_threads_ = 0;
};

}  // namespace octo_planner3d
```

**注意:** 不使用 `costmap_ros`（OctoPlanner 使用自己的 OctoMap），在 `configure()` 中忽略它。

---

## Step 2: 创建 GlobalPlanner 插件实现文件

**文件:** `planner/src/octo_planner_global_planner.cpp`

核心逻辑：

### `configure()` 步骤:
1. 从 `parent` lifecycle node 获取参数（通过 `parent.lock()->declare_parameter()` 或通过外部 params.yaml + nav2 node overrides）
2. 创建 `global_planner::GlobalPlanner` 实例
3. 读取 PCD 文件路径（在 `nav2_params_3d.yaml` 中配置 `pcd_file_path`），调用 `Pcd2OctomapConverter` 构建 OctoMap
4. 将 OctoMap set 到 planner 中
5. 设置所有规划参数

**关键点:** PCD 文件路径不能硬编码，从参数获取，默认路径可以是现有的 PCD 文件路径。

### `createPlan()` 步骤:
1. 接收 `start` 和 `goal` 作为 `PoseStamped`
2. 转换为 `global_planner::PointPose`
3. 调用 `planner_->makePlan(start, goal)` + `planner_->smoothPath()` + `planner_->getPlannerResults()`
4. 将 `vector<PointPose>` 转换为 `nav_msgs::msg::Path`（带朝向插值）
5. 返回 path

### `activate()/deactivate()/cleanup()`:
标准 lifecycle 实现，activate 不做特殊操作（planner 已经 ready），cleanup 释放资源。

---

## Step 3: 创建 plugin.xml

**文件:** `plugin.xml`

```xml
<library path="octo_planner_global_planner">
  <class name="octo_planner3d/OctoPlannerGlobalPlanner"
         type="octo_planner3d::OctoPlannerGlobalPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>3D OctoMap-based global planner with ESDF traversability analysis</description>
  </class>
</library>
```

---

## Step 4: 修改 CMakeLists.txt

1. 添加 `find_package(nav2_core REQUIRED)` 和 `find_package(pluginlib REQUIRED)`
2. 新建共享库 `octo_planner_global_planner`:
   - 源文件: `planner/src/octo_planner_global_planner.cpp`
   - 链接: `GlobalPlanner`, `Octomap`
3. ament_target_dependencies 添加 `nav2_core`, `pluginlib`
4. 在 plugin.xml 安装中导出

---

## Step 5: 修改 package.xml

添加依赖:
```xml
<depend>nav2_core</depend>
<depend>nav2_costmap_2d</depend>
<depend>pluginlib</depend>
```

---

## Step 6: 从 octo_planner_rviz_node 移除 Action Server

**文件:** `src/octo_planner_rviz_node.cpp`

移除内容:
1. `#include <nav2_msgs/action/compute_path_to_pose.hpp>` 和 `#include <rclcpp_action/rclcpp_action.hpp>`
2. `using ComputePathToPose = ...`
3. Action server 创建代码（`create_server<...>`）
4. `handleActionGoal()`, `handleActionCancel()`, `handleActionAccepted()`, `executeComputePath()` 方法
5. `compute_path_action_server_` 成员变量
6. `#include <thread>` （不再需要线程 detach）

保留内容（不受影响）:
- 所有可视化发布器、marker
- PCD → OctoMap 转换流程
- `planNavPath()`（用于 `initialpose` 模式手动调试）
- `planTestPath()`（用于 Publish Point 测试）
- 地图发布、voxel 缓存
- `path_pub_` (发布到 `planned_path` topic 用于 RViz 可视化)

注意：`octo_planner_rviz_node` 仍作为独立节点启动，负责地图可视化 + 测试模式（Publish Point），但不提供 `compute_path_to_pose` action。Nav2 的规划由 `planner_server` 加载 `OctoPlannerGlobalPlanner` 插件处理。

---

## Step 7: 更新 nav2_params_3d.yaml

**文件:** `nav2_dog_slam/config/nav2_params_3d.yaml`

添加 `planner_server` 配置段：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["OctoPlanner3D"]

    OctoPlanner3D:
      plugin: "octo_planner3d/OctoPlannerGlobalPlanner"
      # 传递给插件的参数在插件自己的 configure() 中通过 declare_parameter 读取
      input_pcd: "/home/ztl/slam_data/3d_map/3dmap.pcd"
      output_bt: ""
      resolution: 0.2
      min_points_per_voxel: 2
      min_cluster_voxels: 2
      openmp_num_threads: 0
      robot_radius: 0.25
      max_iterations: 800000
      snap_search_radius_cells: 12
      require_ground_support: true
      strict_direct_ground_support: false
      ground_support_xy_radius_cells: 1
      ground_support_depth_cells: 1
      enable_preblocked_costmap: true
      preblocked_costmap_radius_cells: 3
      preblocked_costmap_weight: 2.5
      lowest_traversable_only: false
      smoothing_enabled: true
      smoothing_simplify_epsilon: 0.1
      smoothing_interp_spacing: 0.15
      smoothing_gradient_iterations: 50
      smoothing_gradient_alpha: 0.3
```

更新 `lifecycle_manager` 节点列表：
```yaml
node_names: ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']
```

更新注释，删除"移除了planner_server"相关说明。

---

## Step 8: 更新启动文件

### navigation_launch_3d.py
添加 `planner_server` 节点启动（与 `controller_server` 同级）：
```python
Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[configured_params_local],
    prefix=['taskset -c 4'],
    remappings=remappings),
```

### lio_nav2_unified_3d.launch.py
1. `lifecycle_nodes` 列表添加 `'planner_server'`
2. OctoPlanner 节点的 remappings 不再需要 `planned_path`（由 planner_server 管理）
3. 可选：保留 octo_planner_rviz_node 启动（仅用于地图可视化和手动测试），或移除其启动

---

## Step 9: 编译验证

```bash
cd ~/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
colcon build --packages-select octo_planner3d nav2_dog_slam
```

---

## 注意事项

1. **PCD 路径**: `configure()` 中通过参数获取，默认值使用 `params.yaml` 中的当前值
2. **OctoMap 只在 configure 时构建一次**（PCD → OctoMap 转换是重量级操作），不支持运行时换地图
3. **不使用 costmap**: 传统 Nav2 planner（如 Smac）依赖 2D costmap，但 OctoPlanner 使用自己的 3D OctoMap
4. **无路径缓存**: `createPlan()` 每次都重新规划（A* 搜索），适合全局规划和重规划场景
5. **octo_planner_rviz_node 仍然独立启动**: 用于地图可视化 + Publish Point 测试模式，但不再提供 action server
6. **`planned_path` topic**: 现在由 `planner_server` 管理，topic 由 Nav2 remappings 定义（通常 `/plan`），不再需要显式 remap 到 `/planned_path`
