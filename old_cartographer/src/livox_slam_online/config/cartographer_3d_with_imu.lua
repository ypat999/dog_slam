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
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
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
}

-- 针对Livox MID-360点云数据优化
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  -- Livox数据已经是完整的点云
TRAJECTORY_BUILDER_3D.min_range = 1  -- 最小距离
TRAJECTORY_BUILDER_3D.max_range = 200.0  -- 最大距离
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1  -- 体素滤波器大小

-- 高分辨率自适应体素滤波器
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 1.0,
  min_num_points = 100,
  max_range = 10.0,
}

-- 低分辨率自适应体素滤波器
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter = {
  max_length = 2.0,
  min_num_points = 150,
  max_range = 50.0,
}

-- 实时相关扫描匹配器
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.1,
  angular_search_window = math.rad(0.5),
  translation_delta_cost_weight = 1e-1,
  rotation_delta_cost_weight = 1e-1,
}

-- Ceres扫描匹配器
TRAJECTORY_BUILDER_3D.ceres_scan_matcher = {
  occupied_space_weight_0 = 1.0,
  occupied_space_weight_1 = 6.0,
  translation_weight = 5.0,
  rotation_weight = 4e2,
  only_optimize_yaw = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 10,
    num_threads = 1,
  },
}

-- 运动滤波器
TRAJECTORY_BUILDER_3D.motion_filter = {
  max_time_seconds = 0.5,
  max_distance_meters = 0.1,
  max_angle_radians = math.rad(1.),
}

-- 亚像素对齐
TRAJECTORY_BUILDER_3D.submaps = {
  high_resolution = 0.10,
  high_resolution_max_range = 20.0,
  low_resolution = 0.45,
  num_range_data = 1000,  -- 每个子图的点云数量
  range_data_inserter = {
    hit_probability = 0.55,
    miss_probability = 0.49,
    num_free_space_voxels = 2,
  },
}

-- IMU配置
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0


-- 使用3D轨迹构建器
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

-- 位姿图优化参数
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 90  -- 更频繁的优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03

-- 位姿图优化问题配置
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options