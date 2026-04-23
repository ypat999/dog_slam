#include "traversability_layer/traversability_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer, nav2_costmap_2d::Layer)

namespace traversability_layer
{

TraversabilityLayer::TraversabilityLayer()
: height_map_size_x_(0),
  height_map_size_y_(0),
  height_map_valid_(false)
{
}

TraversabilityLayer::~TraversabilityLayer()
{
}

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: Unable to lock node!");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(std::string("/cloud_registered_body")));
  declareParameter("sensor_frame", rclcpp::ParameterValue(std::string("")));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(-0.5));
  declareParameter("max_slope_traversable", rclcpp::ParameterValue(45.0));
  declareParameter("step_height_threshold", rclcpp::ParameterValue(0.15));
  declareParameter("slope_cost_scale", rclcpp::ParameterValue(2.0));
  declareParameter("height_cost_scale", rclcpp::ParameterValue(3.0));
  declareParameter("lethal_cost_threshold", rclcpp::ParameterValue(254.0));
  declareParameter("cloud_buffer_size", rclcpp::ParameterValue(5));
  declareParameter("publish_slope_map", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic_);
  node->get_parameter(name_ + ".sensor_frame", sensor_frame_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_slope_traversable", max_slope_traversable_);
  node->get_parameter(name_ + ".step_height_threshold", step_height_threshold_);
  node->get_parameter(name_ + ".slope_cost_scale", slope_cost_scale_);
  node->get_parameter(name_ + ".height_cost_scale", height_cost_scale_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(name_ + ".cloud_buffer_size", cloud_buffer_size_);
  node->get_parameter(name_ + ".publish_slope_map", publish_slope_map_);

  max_slope_traversable_ = max_slope_traversable_ * M_PI / 180.0;

  RCLCPP_INFO(
    node->get_logger(),
    "TraversabilityLayer: step_height_threshold=%.3f, max_slope_traversable=%.1f deg, "
    "pointcloud_topic=%s",
    step_height_threshold_, max_slope_traversable_ * 180.0 / M_PI,
    pointcloud_topic_.c_str());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS cloud_qos(rclcpp::KeepLast(static_cast<size_t>(cloud_buffer_size_)));
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, cloud_qos,
    std::bind(&TraversabilityLayer::pointCloudCallback, this, std::placeholders::_1));

  if (publish_slope_map_) {
    slope_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "slope_map", rclcpp::QoS(1));
  }

  current_ = true;
}

void TraversabilityLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) {
    return;
  }

  std::string target_frame = layered_costmap_->getGlobalFrameID();
  std::string source_frame = msg->header.frame_id;

  if (!sensor_frame_.empty()) {
    source_frame = sensor_frame_;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("traversability_layer"),
      "TraversabilityLayer: TF transform failed: %s", ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_cloud->reserve(cloud->size());

  double tx = transform.transform.translation.x;
  double ty = transform.transform.translation.y;
  double tz = transform.transform.translation.z;
  double qx = transform.transform.rotation.x;
  double qy = transform.transform.rotation.y;
  double qz = transform.transform.rotation.z;
  double qw = transform.transform.rotation.w;

  double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  double r01 = 2.0 * (qx * qy - qz * qw);
  double r02 = 2.0 * (qx * qz + qy * qw);
  double r10 = 2.0 * (qx * qy + qz * qw);
  double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  double r12 = 2.0 * (qy * qz - qx * qw);
  double r20 = 2.0 * (qx * qz - qy * qw);
  double r21 = 2.0 * (qy * qz + qx * qw);
  double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    double z_local = r20 * pt.x + r21 * pt.y + r22 * pt.z + tz;
    if (z_local > max_obstacle_height_ || z_local < min_obstacle_height_) {
      continue;
    }

    double x_global = r00 * pt.x + r01 * pt.y + r02 * pt.z + tx;
    double y_global = r10 * pt.x + r11 * pt.y + r12 * pt.z + ty;

    transformed_cloud->push_back(
      pcl::PointXYZ(
        static_cast<float>(x_global),
        static_cast<float>(y_global),
        static_cast<float>(z_local)));
  }

  processCloud(transformed_cloud);
}

void TraversabilityLayer::processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();

  if (size_x == 0 || size_y == 0) {
    return;
  }

  if (height_map_.size() != static_cast<size_t>(size_x) * size_y) {
    height_map_.resize(static_cast<size_t>(size_x) * size_y);
    height_map_size_x_ = size_x;
    height_map_size_y_ = size_y;
  }

  for (auto & cell : height_map_) {
    cell.min_z = std::numeric_limits<float>::max();
    cell.max_z = std::numeric_limits<float>::lowest();
    cell.mean_z = 0.0f;
    cell.point_count = 0;
    cell.height_diff = 0.0f;
    cell.slope_x = 0.0f;
    cell.slope_y = 0.0f;
    cell.slope_magnitude = 0.0f;
    cell.has_data = false;
  }

  double ox = getOriginX();
  double oy = getOriginY();
  double res = getResolution();

  for (const auto & pt : cloud->points) {
    int mx = static_cast<int>((pt.x - ox) / res);
    int my = static_cast<int>((pt.y - oy) / res);

    if (mx < 0 || mx >= static_cast<int>(size_x) ||
        my < 0 || my >= static_cast<int>(size_y))
    {
      continue;
    }

    auto & cell = height_map_[static_cast<size_t>(mx) + static_cast<size_t>(my) * size_x];

    if (pt.z < cell.min_z) cell.min_z = pt.z;
    if (pt.z > cell.max_z) cell.max_z = pt.z;
    cell.mean_z += pt.z;
    cell.point_count++;
    cell.has_data = true;
  }

  for (auto & cell : height_map_) {
    if (cell.has_data && cell.point_count > 0) {
      cell.mean_z /= static_cast<float>(cell.point_count);
      cell.height_diff = cell.max_z - cell.min_z;
    }
  }

  computeSlope();
  height_map_valid_ = true;
}

void TraversabilityLayer::computeSlope()
{
  unsigned int size_x = height_map_size_x_;
  unsigned int size_y = height_map_size_y_;
  double res = getResolution();

  for (unsigned int mx = 1; mx < size_x - 1; mx++) {
    for (unsigned int my = 1; my < size_y - 1; my++) {
      auto & cell = height_map_[mx + my * size_x];
      if (!cell.has_data || cell.point_count < 2) {
        continue;
      }

      auto & cell_xp = height_map_[(mx + 1) + my * size_x];
      auto & cell_xm = height_map_[(mx - 1) + my * size_x];
      auto & cell_yp = height_map_[mx + (my + 1) * size_x];
      auto & cell_ym = height_map_[mx + (my - 1) * size_x];

      float dz_dx = 0.0f;
      float dz_dy = 0.0f;
      int count_x = 0;
      int count_y = 0;

      if (cell_xp.has_data && cell_xm.has_data) {
        dz_dx = (cell_xp.mean_z - cell_xm.mean_z) / (2.0f * static_cast<float>(res));
        count_x = 2;
      } else if (cell_xp.has_data) {
        dz_dx = (cell_xp.mean_z - cell.mean_z) / static_cast<float>(res);
        count_x = 1;
      } else if (cell_xm.has_data) {
        dz_dx = (cell.mean_z - cell_xm.mean_z) / static_cast<float>(res);
        count_x = 1;
      }

      if (cell_yp.has_data && cell_ym.has_data) {
        dz_dy = (cell_yp.mean_z - cell_ym.mean_z) / (2.0f * static_cast<float>(res));
        count_y = 2;
      } else if (cell_yp.has_data) {
        dz_dy = (cell_yp.mean_z - cell.mean_z) / static_cast<float>(res);
        count_y = 1;
      } else if (cell_ym.has_data) {
        dz_dy = (cell.mean_z - cell_ym.mean_z) / static_cast<float>(res);
        count_y = 1;
      }

      if (count_x > 0 || count_y > 0) {
        cell.slope_x = dz_dx;
        cell.slope_y = dz_dy;
        cell.slope_magnitude = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
      }
    }
  }
}

unsigned char TraversabilityLayer::computeCost(const CellData & cell) const
{
  if (!cell.has_data || cell.point_count < 2) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  if (cell.height_diff > step_height_threshold_) {
    float height_excess = cell.height_diff - step_height_threshold_;
    float height_cost = height_excess * height_cost_scale_ * 100.0f;
    if (height_cost >= lethal_cost_threshold_) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    return static_cast<unsigned char>(
      std::min(static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE),
               height_cost));
  }

  float slope_angle = std::atan(cell.slope_magnitude);

  if (slope_angle > max_slope_traversable_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (slope_angle < 1e-6f) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  float slope_ratio = slope_angle / max_slope_traversable_;
  float slope_cost = slope_ratio * slope_cost_scale_ * 100.0f;
  float height_cost = cell.height_diff * height_cost_scale_ * 100.0f;

  float total_cost = slope_cost + height_cost;

  if (total_cost >= lethal_cost_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  return static_cast<unsigned char>(
    std::min(static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE),
             std::max(static_cast<float>(nav2_costmap_2d::FREE_SPACE),
                      total_cost)));
}

void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!height_map_valid_) {
    return;
  }

  double ox = getOriginX();
  double oy = getOriginY();
  double res = getResolution();
  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();

  for (unsigned int mx = 0; mx < size_x; mx++) {
    for (unsigned int my = 0; my < size_y; my++) {
      const auto & cell = height_map_[mx + my * size_x];
      if (!cell.has_data) {
        continue;
      }

      double wx = ox + (mx + 0.5) * res;
      double wy = oy + (my + 0.5) * res;

      if (cell.height_diff > step_height_threshold_ ||
          cell.slope_magnitude > std::tan(max_slope_traversable_)) {
        *min_x = std::min(*min_x, wx - res);
        *min_y = std::min(*min_y, wy - res);
        *max_x = std::max(*max_x, wx + res);
        *max_y = std::max(*max_y, wy + res);
      }
    }
  }
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!height_map_valid_) {
    return;
  }

  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();

  unsigned char * master_array = master_grid.getCharMap();

  for (int mx = min_i; mx < max_i; mx++) {
    for (int my = min_j; my < max_j; my++) {
      if (mx < 0 || mx >= static_cast<int>(size_x) ||
          my < 0 || my >= static_cast<int>(size_y))
      {
        continue;
      }

      const auto & cell = height_map_[static_cast<unsigned int>(mx) +
                                       static_cast<unsigned int>(my) * size_x];
      if (!cell.has_data) {
        continue;
      }

      unsigned char cost = computeCost(cell);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      unsigned int idx = master_grid.getIndex(mx, my);
      unsigned char old_cost = master_array[idx];

      if (old_cost == nav2_costmap_2d::NO_INFORMATION ||
          cost > old_cost) {
        master_array[idx] = cost;
      }
    }
  }

  if (publish_slope_map_ && slope_pub_) {
    auto node = node_.lock();
    if (node) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      slope_cloud->header.frame_id = layered_costmap_->getGlobalFrameID();

      double ox = getOriginX();
      double oy = getOriginY();
      double res = getResolution();

      for (unsigned int mx = 0; mx < size_x; mx++) {
        for (unsigned int my = 0; my < size_y; my++) {
          const auto & cell = height_map_[mx + my * size_x];
          if (!cell.has_data) {
            continue;
          }

          pcl::PointXYZI pt;
          pt.x = static_cast<float>(ox + (mx + 0.5) * res);
          pt.y = static_cast<float>(oy + (my + 0.5) * res);
          pt.z = cell.mean_z;
          pt.intensity = cell.slope_magnitude;
          slope_cloud->push_back(pt);
        }
      }

      sensor_msgs::msg::PointCloud2 slope_msg;
      pcl::toROSMsg(*slope_cloud, slope_msg);
      slope_msg.header.stamp = node->now();
      slope_pub_->publish(slope_msg);
    }
  }
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
}

void TraversabilityLayer::resetMaps()
{
  height_map_.clear();
  height_map_valid_ = false;
  height_map_size_x_ = 0;
  height_map_size_y_ = 0;
}

void TraversabilityLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  rclcpp::QoS cloud_qos(rclcpp::KeepLast(static_cast<size_t>(cloud_buffer_size_)));
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, cloud_qos,
    std::bind(&TraversabilityLayer::pointCloudCallback, this, std::placeholders::_1));

  if (publish_slope_map_) {
    slope_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "slope_map", rclcpp::QoS(1));
  }
}

void TraversabilityLayer::deactivate()
{
  cloud_sub_.reset();
  slope_pub_.reset();
}

}  // namespace traversability_layer
