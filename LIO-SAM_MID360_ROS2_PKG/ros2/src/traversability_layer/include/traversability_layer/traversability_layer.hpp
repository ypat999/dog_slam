#pragma once

#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace traversability_layer
{

struct Point3D
{
  double x;
  double y;
  double z;
};

struct VoxelData
{
  uint8_t hit_count = 0;
  uint8_t pass_count = 0;
  uint16_t remaining_uses = 0;  // 次数制衰减：剩余参与计算次数，0=过期
};

struct GroundCell
{
  float ground_z = 0.0f;
  bool has_ground = false;
  bool is_interpolated = false;
  float height_diff = 0.0f;
  float slope_x = 0.0f;
  float slope_y = 0.0f;
  float slope_magnitude = 0.0f;
  float obstacle_ratio = 0.0f;
  float max_obstacle_z = 0.0f;
  float min_obstacle_z = 0.0f;
};

struct IncrementalRay
{
  size_t hit_idx;
  std::vector<size_t> pass_indices;
};

struct PersistentCostCell
{
  unsigned char cost = 255;  // 255 = NO_INFORMATION = 从未观测
};

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer();
  virtual ~TraversabilityLayer();

  void onInitialize() override;
  void matchSize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override { return true; }

  void activate() override;
  void deactivate() override;

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void updateVoxelGrid(
    const std::vector<Point3D> & transformed_pts,
    const Point3D & sensor_pos);
  void shiftVoxelGrid(int shift_x, int shift_y);
  void shiftGroundMap(int shift_x, int shift_y);
  void expandVoxelGridZ(double new_z_lo, double new_z_hi);
  void tickVoxelGrid();  // 次数制衰减（替代 decayVoxelGrid）
  void shiftPersistentCostMap(int shift_x, int shift_y);
  void extractGroundInCache();
  void interpolateGround();
  void computeGroundSlope();
  unsigned char computeCost(const GroundCell & cell) const;
  void resetMaps();

  inline size_t voxelIndex(unsigned int ix, unsigned int iy, unsigned int iz) const
  {
    return static_cast<size_t>(iz) * voxel_size_x_ * voxel_size_y_ +
           static_cast<size_t>(iy) * voxel_size_x_ +
           static_cast<size_t>(ix);
  }

  inline size_t groundIndex(unsigned int cx, unsigned int cy) const
  {
    return static_cast<size_t>(cy) * ground_size_x_ + static_cast<size_t>(cx);
  }

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string pointcloud_topic_;
  std::string sensor_frame_;
  std::string base_frame_;
  double max_obstacle_height_;
  double min_obstacle_height_;
  double max_slope_traversable_;
  double slope_cost_start_;
  double step_height_threshold_;
  double height_cost_start_;
  double slope_cost_scale_;
  double height_cost_scale_;
  double lethal_cost_threshold_;
  int observation_persistence_int_;  // 次数制衰减次数（0=仅最新帧，>0=体素存留计算次数）
  int cloud_buffer_size_;
  bool enabled_;
  bool publish_slope_map_;
  double cell_resolution_;
  int num_threads_;

  double voxel_z_resolution_;
  int ground_hit_threshold_;
  int free_space_threshold_;
  int free_space_window_;
  int interp_search_radius_;
  int min_interp_neighbors_;
  double obstacle_ratio_threshold_;
  int obstacle_hit_threshold_;

  // 新增参数
  int skip_frames_;                     // 跳帧：N帧取1帧计算cost，0=不跳帧
  bool persist_cost_;                   // 永久cost记忆开关
  bool trust_interpolated_ground_;      // 信任插值地面开关
  bool enable_perf_log_;                // 性能统计开关

  // 性能统计（每30秒输出各模块平均耗时）
  rclcpp::Time last_perf_log_{0, 0, RCL_ROS_TIME};
  int perf_frame_count_ = 0;              // 点云回调帧数（仅计算帧）
  int perf_cost_frame_count_ = 0;         // updateCosts帧数
  double perf_cloud_transform_ms_ = 0.0;  // 点云坐标变换+滤波
  double perf_voxel_grid_ms_ = 0.0;       // 体素更新（updateVoxelGrid）
  double perf_tick_voxel_ms_ = 0.0;       // 体素衰减（tickVoxelGrid）
  double perf_extract_ground_ms_ = 0.0;   // 地面提取（extractGroundInCache）
  double perf_interpolate_ms_ = 0.0;      // 地面插值（interpolateGround）
  double perf_slope_ms_ = 0.0;            // 坡度计算（computeGroundSlope）
  double perf_costmap_ms_ = 0.0;          // 代价图更新（updateCosts）
  double perf_total_ms_ = 0.0;            // 完整流水线总耗时（仅计算帧）

  std::vector<VoxelData> voxel_grid_;
  unsigned int voxel_size_x_ = 0;
  unsigned int voxel_size_y_ = 0;
  unsigned int voxel_size_z_ = 0;
  double voxel_z_origin_ = 0.0;
  double voxel_ox_ = 0.0;  // 缓存网格左下角 x 坐标（odom 坐标系）
  double voxel_oy_ = 0.0;  // 缓存网格左下角 y 坐标（odom 坐标系）
  bool voxel_grid_valid_ = false;
  uint16_t frame_counter_ = 0;
  uint32_t compute_counter_ = 0;  // 计算帧计数（跳帧时 != frame_counter_）
  uint16_t decay_interval_frames_ = 10;
  uint32_t cloud_received_ = 0;
  uint32_t cloud_processed_ = 0;

  std::vector<GroundCell> ground_map_;
  unsigned int ground_size_x_ = 0;  // 缓存网格 = costmap + 安全边界
  unsigned int ground_size_y_ = 0;
  int half_margin_ = 0;             // 半边界 cells，用于缓存居中计算

  // 永久 cost 记忆
  std::vector<PersistentCostCell> persistent_cost_map_;

  unsigned int costmap_size_x_ = 0;  // costmap 原始 x 大小
  unsigned int costmap_size_y_ = 0;  // costmap 原始 y 大小
  double costmap_ox_ = 0.0;  // 当前 costmap 原点 x
  double costmap_oy_ = 0.0;  // 当前 costmap 原点 y
  double costmap_res_ = 0.0;  // costmap 分辨率

  double sensor_global_x_ = 0.0;
  double sensor_global_y_ = 0.0;
  double sensor_global_z_ = 0.0;
  double base_global_z_ = 0.0;
  bool cloud_updated_ = false;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace traversability_layer
