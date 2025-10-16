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
  tracking_frame = "livox_frame",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 3D轨迹构建器配置，针对Livox Mid-360优化
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 40.
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)-- Ceres扫描匹配器 - 抑制漂移
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10.0

-- IMU配置
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0


TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

-- 运动滤波器配置 - 抑制平移漂移
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 10000
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(1.0)

-- 子地图配置
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 30

-- 位姿图优化配置 - 抑制漂移
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.matcher_translation_weight = 10.0
POSE_GRAPH.matcher_rotation_weight = 10.0
POSE_GRAPH.global_sampling_ratio = 0.05

-- 地图构建器配置
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4
return options