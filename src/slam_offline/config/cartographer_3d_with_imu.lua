-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "livox_frame",  -- 使用LiDAR坐标系作为跟踪帧
  published_frame = "odom",        -- 发布odom帧，避免与base_link冲突
  odom_frame = "odom",
  provide_odom_frame = true,       -- 提供odom帧，让Cartographer发布map->odom
  publish_frame_projected_to_2d = true, -- 禁用2D投影发布
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 我们只有一个点云输入
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,  -- 保持所有点云数据
  odometry_sampling_ratio = 0.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,  -- 启用IMU数据
  landmarks_sampling_ratio = 0.,
  lookup_transform_timeout_sec = 5,  -- 增加超时时间，提高稳定性
}

-- 针对Livox MID-360点云数据优化 - 减少晃动影响
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  -- Livox数据已经是完整的点云
TRAJECTORY_BUILDER_3D.min_range = 0.3  -- 增加最小距离，过滤近处噪声
TRAJECTORY_BUILDER_3D.max_range = 5.0  -- 增加最大距离，利用更多远距信息
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.08  -- 减小体素滤波器大小，保留更多细节


-- 高度过滤配置 - 忽略一定高度以上的点云
-- TRAJECTORY_BUILDER_2D.min_z = -0.3  -- 最小高度（米），低于此值的点云将被忽略
-- TRAJECTORY_BUILDER_2D.max_z = 1.5   -- 最大高度（米），高于此值的点云将被忽略

-- 高分辨率自适应体素滤波器 - 优化以减少噪声
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 0.8,     -- 减小最大长度，提高分辨率
  min_num_points = 150, -- 增加最小点数，减少噪声
  max_range = 15.0,     -- 增加最大范围
}

-- 低分辨率自适应体素滤波器 - 保持稳定
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
  max_length = 1.5,     -- 减小最大长度
  min_num_points = 200, -- 增加最小点数
  max_range = 40.0,     -- 减小最大范围，避免过远噪声
}

-- 实时相关扫描匹配器 - 优化以减少点云晃动影响
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.05,      -- 减小线性搜索窗口，降低噪声敏感度
  angular_search_window = math.rad(0.3),  -- 减小角度搜索窗口
  translation_delta_cost_weight = 2e-1,   -- 增加平移成本权重
  rotation_delta_cost_weight = 2e-1,      -- 增加旋转成本权重
}

-- Ceres扫描匹配器 - 优化权重以减少晃动误差
TRAJECTORY_BUILDER_3D.ceres_scan_matcher = {
  occupied_space_weight_0 = 1.5,    -- 增加占用空间权重0
  occupied_space_weight_1 = 8.0,    -- 增加占用空间权重1
  translation_weight = 8.0,         -- 增加平移权重，提高位置精度
  rotation_weight = 3e2,            -- 降低旋转权重，避免过度旋转
  only_optimize_yaw = false,
  intensity_cost_function_options_0 = {
    weight = 0.5,
    huber_scale = 0.3,
    intensity_threshold = 0.5,      -- 强度阈值，过滤低强度点云（Humble版本新增参数）
  },
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 15,        -- 增加迭代次数以提高收敛性
    num_threads = 1,
  },
}

-- 运动滤波器 - 严格过滤以减少微小晃动
TRAJECTORY_BUILDER_3D.motion_filter = {
  max_time_seconds = 0.3,           -- 减小时间阈值，更频繁地过滤
  max_distance_meters = 0.05,       -- 减小距离阈值，过滤微小位移
  max_angle_radians = math.rad(0.5), -- 减小角度阈值，过滤微小旋转
}

-- 亚像素对齐 - 优化子图参数以减少晃动并避免纹理问题
TRAJECTORY_BUILDER_3D.submaps = {
  high_resolution = 0.10,           -- 保持适中的高分辨率
  high_resolution_max_range = 20.0,  -- 保持合理的高分辨率范围
  low_resolution = 0.45,            -- 保持适中的低分辨率
  num_range_data = 1200,            -- 增加每个子图的点云数量，避免过早修剪
  range_data_inserter = {
    hit_probability = 0.55,         -- 保持标准命中概率
    miss_probability = 0.49,        -- 保持标准未命中概率
    num_free_space_voxels = 2,      -- 保持标准的自由空间体素数量
    intensity_threshold = 0.5,      -- 强度阈值，过滤低强度点云（Humble版本新增参数）
  },
}


-- IMU配置 - 优化以减少晃动影响
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 9.8  -- 标准重力时间常量

-- 使用3D轨迹构建器 - 优化子图管理
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

-- 轨迹构建器选项 - 避免过早修剪
TRAJECTORY_BUILDER_OPTIONS = {
  max_submaps_to_keep = 5,            -- 保持更多子图，避免过早修剪
  num_multi_echo_laser_scans = 0,
  num_laser_scans = 0,
  num_point_clouds = 1,
}

-- 位姿图优化参数 - 优化以减少累积误差并避免子图问题
POSE_GRAPH.optimization_problem.huber_scale = 4e2      -- 适中的Huber尺度
POSE_GRAPH.optimize_every_n_nodes = 80                 -- 适中的优化频率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03    -- 标准采样比例

-- 位姿图优化问题配置 - 平衡约束质量和稳定性
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12  -- 适中的迭代次数
POSE_GRAPH.constraint_builder.min_score = 0.55        -- 降低最小分数阈值，增加闭环检测机会
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60     -- 降低全局定位最小分数

-- 闭环检测参数 - 增强闭环检测能力
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 4e2      -- 闭环旋转权重
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.5e3  -- 闭环平移权重
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05   -- 提高采样比例，增加闭环检测机会

-- 基于分支定界的闭环检测 - 提高闭环检测精度
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d = {
  linear_xy_search_window = 0.8,   -- XY线性搜索窗口
  linear_z_search_window = 0.3,    -- Z线性搜索窗口
  angular_search_window = math.rad(20.0),  -- 角度搜索窗口（20度）
  branch_and_bound_depth = 8,      -- 分支定界深度
  full_resolution_depth = 3,       -- 全分辨率深度
  min_rotational_score = 0.77,     -- 最小旋转分数
  min_low_resolution_score = 0.55, -- 最小低分辨率分数
}

-- 全局闭环检测优化 - 增强全局定位能力
POSE_GRAPH.global_constraint_search_after_n_seconds = 20  -- 20秒后开始全局约束搜索
POSE_GRAPH.global_sampling_ratio = 0.003  -- 全局采样比例
POSE_GRAPH.max_num_final_iterations = 100  -- 增加最终迭代次数
POSE_GRAPH.matcher_translation_weight = 1e2  -- 匹配器平移权重
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 匹配器旋转权重


return options