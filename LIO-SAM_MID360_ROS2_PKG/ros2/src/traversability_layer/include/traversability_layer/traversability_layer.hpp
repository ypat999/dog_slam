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

struct CellData
{
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  float representative_z = 0.0f;
  unsigned int point_count = 0;
  float height_diff = 0.0f;
  float slope_x = 0.0f;
  float slope_y = 0.0f;
  float slope_magnitude = 0.0f;
  bool has_data = false;
};

struct TimedPoint
{
  Point3D pt;
  rclcpp::Time stamp;
};

class TraversabilityLayer : public nav2_costmap_2d::Layer, public nav2_costmap_2d::Costmap2D
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
  void computeSlope(double origin_x, double origin_y);
  unsigned char computeCost(const CellData & cell) const;
  void resetMaps();

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string pointcloud_topic_;
  std::string sensor_frame_;
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
  bool enable_raycast_clear_;
  double cell_resolution_;
  int num_threads_;

  rclcpp::Time last_perf_log_{0, 0, RCL_ROS_TIME};
  int perf_frame_count_ = 0;
  double perf_total_time_ = 0.0;

  std::vector<CellData> grid_map_;
  unsigned int grid_size_x_ = 0;
  unsigned int grid_size_y_ = 0;

  std::vector<TimedPoint> accumulated_cloud_;
  double sensor_global_x_ = 0.0;
  double sensor_global_y_ = 0.0;
  bool cloud_updated_ = false;

  inline size_t gridIndex(unsigned int cx, unsigned int cy) const
  {
    return static_cast<size_t>(cy) * grid_size_x_ + static_cast<size_t>(cx);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace traversability_layer
