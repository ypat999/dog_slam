#pragma once

#include <mutex>
#include <vector>
#include <unordered_map>
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

struct CellData
{
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  float mean_z = 0.0f;
  unsigned int point_count = 0;
  float height_diff = 0.0f;
  float slope_x = 0.0f;
  float slope_y = 0.0f;
  float slope_magnitude = 0.0f;
  bool has_data = false;
  rclcpp::Time last_seen_time{0, 0, RCL_ROS_TIME};
  bool ray_cleared = false;
};

struct GridKey
{
  int32_t gx;
  int32_t gy;

  bool operator==(const GridKey & other) const
  {
    return gx == other.gx && gy == other.gy;
  }
};

struct GridKeyHasher
{
  size_t operator()(const GridKey & k) const
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(k.gx)) << 32) ^
           static_cast<uint32_t>(k.gy);
  }
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
  void processCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const rclcpp::Time & stamp);
  void raycastClear(
    double sensor_x, double sensor_y,
    const std::vector<std::pair<int, int>> & marked_cells);
  void computeSlope();
  unsigned char computeCost(const CellData & cell) const;
  void resetMaps();
  void decayOldObservations(const rclcpp::Time & now);

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string pointcloud_topic_;
  std::string sensor_frame_;
  double max_obstacle_height_;
  double min_obstacle_height_;
  double max_slope_traversable_;
  double slope_cost_start_;
  double step_height_threshold_;
  double slope_cost_scale_;
  double height_cost_scale_;
  double lethal_cost_threshold_;
  double observation_persistence_;
  double clearing_duration_;
  int cloud_buffer_size_;
  bool enabled_;
  bool publish_slope_map_;
  bool enable_raycast_clear_;
  bool clear_each_frame_;

  std::unordered_map<GridKey, CellData, GridKeyHasher> global_height_map_;

  inline void worldToGrid(double wx, double wy, int32_t & gx, int32_t & gy) const
  {
    const double res = getResolution();
    gx = static_cast<int32_t>(std::floor(wx / res));
    gy = static_cast<int32_t>(std::floor(wy / res));
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace traversability_layer
