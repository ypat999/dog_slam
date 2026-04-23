#pragma once

#include <mutex>
#include <vector>
#include <memory>
#include <string>

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
};

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer();
  virtual ~TraversabilityLayer();

  void onInitialize() override;
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
  void processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void computeSlope();
  unsigned char computeCost(const CellData & cell) const;
  void resetMaps();

  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string pointcloud_topic_;
  std::string sensor_frame_;
  double max_obstacle_height_;
  double min_obstacle_height_;
  double max_slope_traversable_;
  double step_height_threshold_;
  double slope_cost_scale_;
  double height_cost_scale_;
  double lethal_cost_threshold_;
  int cloud_buffer_size_;
  bool enabled_;
  bool publish_slope_map_;

  std::vector<CellData> height_map_;
  unsigned int height_map_size_x_;
  unsigned int height_map_size_y_;
  bool height_map_valid_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace traversability_layer
