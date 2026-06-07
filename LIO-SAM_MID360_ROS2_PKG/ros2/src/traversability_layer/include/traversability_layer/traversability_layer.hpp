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
  uint16_t last_update_frame = 0;
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
  void expandVoxelGridZ(double new_z_lo, double new_z_hi);
  void decayVoxelGrid();
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
  double observation_persistence_;
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

  rclcpp::Time last_perf_log_{0, 0, RCL_ROS_TIME};
  int perf_frame_count_ = 0;
  double perf_total_time_ = 0.0;

  std::vector<VoxelData> voxel_grid_;
  unsigned int voxel_size_x_ = 0;
  unsigned int voxel_size_y_ = 0;
  unsigned int voxel_size_z_ = 0;
  double voxel_z_origin_ = 0.0;
  double voxel_ox_ = 0.0;  // 缓存网格左下角 x 坐标（odom 坐标系）
  double voxel_oy_ = 0.0;  // 缓存网格左下角 y 坐标（odom 坐标系）
  bool voxel_grid_valid_ = false;
  uint16_t frame_counter_ = 0;
  uint16_t decay_interval_frames_ = 10;
  uint32_t cloud_received_ = 0;
  uint32_t cloud_processed_ = 0;

  std::vector<GroundCell> ground_map_;
  unsigned int ground_size_x_ = 0;  // 缓存网格 x 方向大小（costmap 2倍）
  unsigned int ground_size_y_ = 0;  // 缓存网格 y 方向大小（costmap 2倍）

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
