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
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(std::string("/lio/body/cloud")));
  declareParameter("sensor_frame", rclcpp::ParameterValue(std::string("")));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(-0.5));
  declareParameter("max_slope_traversable", rclcpp::ParameterValue(45.0));
  declareParameter("slope_cost_start", rclcpp::ParameterValue(15.0));
  declareParameter("step_height_threshold", rclcpp::ParameterValue(0.15));
  declareParameter("slope_cost_scale", rclcpp::ParameterValue(5.0));
  declareParameter("height_cost_scale", rclcpp::ParameterValue(10.0));
  declareParameter("lethal_cost_threshold", rclcpp::ParameterValue(254.0));
  declareParameter("observation_persistence", rclcpp::ParameterValue(5.0));
  declareParameter("clearing_duration", rclcpp::ParameterValue(10.0));
  declareParameter("cloud_buffer_size", rclcpp::ParameterValue(5));
  declareParameter("publish_slope_map", rclcpp::ParameterValue(false));
  declareParameter("enable_raycast_clear", rclcpp::ParameterValue(false));
  declareParameter("clear_each_frame", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic_);
  node->get_parameter(name_ + ".sensor_frame", sensor_frame_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_slope_traversable", max_slope_traversable_);
  node->get_parameter(name_ + ".slope_cost_start", slope_cost_start_);
  node->get_parameter(name_ + ".step_height_threshold", step_height_threshold_);
  node->get_parameter(name_ + ".slope_cost_scale", slope_cost_scale_);
  node->get_parameter(name_ + ".height_cost_scale", height_cost_scale_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(name_ + ".observation_persistence", observation_persistence_);
  node->get_parameter(name_ + ".clearing_duration", clearing_duration_);
  node->get_parameter(name_ + ".cloud_buffer_size", cloud_buffer_size_);
  node->get_parameter(name_ + ".publish_slope_map", publish_slope_map_);
  node->get_parameter(name_ + ".enable_raycast_clear", enable_raycast_clear_);
  node->get_parameter(name_ + ".clear_each_frame", clear_each_frame_);

  max_slope_traversable_ = max_slope_traversable_ * M_PI / 180.0;
  slope_cost_start_ = slope_cost_start_ * M_PI / 180.0;

  RCLCPP_INFO(
    node->get_logger(),
    "TraversabilityLayer: step_height_threshold=%.3f, max_slope_traversable=%.1f deg, "
    "slope_cost_start=%.1f deg, pointcloud_topic=%s, observation_persistence=%.1fs, "
    "clearing_duration=%.1fs, enable_raycast_clear=%s, clear_each_frame=%s",
    step_height_threshold_, max_slope_traversable_ * 180.0 / M_PI,
    slope_cost_start_ * 180.0 / M_PI, pointcloud_topic_.c_str(),
    observation_persistence_, clearing_duration_,
    enable_raycast_clear_ ? "true" : "false",
    clear_each_frame_ ? "true" : "false");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS cloud_qos(rclcpp::KeepLast(static_cast<size_t>(cloud_buffer_size_)));
  cloud_qos.best_effort();
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, cloud_qos,
    std::bind(&TraversabilityLayer::pointCloudCallback, this, std::placeholders::_1));

  if (publish_slope_map_) {
    slope_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "slope_map", rclcpp::QoS(1));
  }

  // 让当前图层的网格尺寸、分辨率、原点与主地图（layered_costmap_）完全对齐
  matchSize();

  current_ = true;
}

void TraversabilityLayer::matchSize()
{
  // 获取主地图的属性
  nav2_costmap_2d::Costmap2D * master_grid = layered_costmap_->getCostmap();
  
  // 设置当前图层自身的 Costmap2D 尺寸
  resizeMap(master_grid->getSizeInCellsX(), master_grid->getSizeInCellsY(), 
            master_grid->getResolution(), 
            master_grid->getOriginX(), master_grid->getOriginY());
}

void TraversabilityLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] Received cloud: %zu points, frame_id=%s",
    cloud->size(), msg->header.frame_id.c_str());

  if (cloud->empty()) {
    RCLCPP_WARN(rclcpp::get_logger("traversability_layer"), "[TraversabilityLayer] Cloud is empty!");
    return;
  }

  std::string target_frame = layered_costmap_->getGlobalFrameID();
  std::string source_frame = msg->header.frame_id;

  if (!sensor_frame_.empty() && sensor_frame_ != "") {
    source_frame = sensor_frame_;
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] TF lookup: target=%s, source=%s",
    target_frame.c_str(), source_frame.c_str());

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("traversability_layer"), *clock, 2000,
      "[TraversabilityLayer] TF success: translation=[%.3f, %.3f, %.3f]",
      transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("traversability_layer"), *clock, 2000,
      "[TraversabilityLayer] TF transform failed: %s (target=%s, source=%s)",
      ex.what(), target_frame.c_str(), source_frame.c_str());
    return;
  }

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_cloud->reserve(cloud->size());

  int filtered_count = 0;
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      filtered_count++;
      continue;
    }

    double x_global = r00 * pt.x + r01 * pt.y + r02 * pt.z + tx;
    double y_global = r10 * pt.x + r11 * pt.y + r12 * pt.z + ty;
    double z_global = r20 * pt.x + r21 * pt.y + r22 * pt.z + tz;

    double z_robot_relative = z_global - tz;
    if (z_robot_relative > max_obstacle_height_ || z_robot_relative < min_obstacle_height_) {
      filtered_count++;
      continue;
    }

    transformed_cloud->push_back(
      pcl::PointXYZ(
        static_cast<float>(x_global),
        static_cast<float>(y_global),
        static_cast<float>(z_global)));
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] After transform: %zu points kept, %d filtered (z_range=[%.2f, %.2f])",
    transformed_cloud->size(), filtered_count, min_obstacle_height_, max_obstacle_height_);

  processCloud(transformed_cloud, msg->header.stamp);
}

void TraversabilityLayer::processCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const rclcpp::Time & stamp)
{
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] processCloud: cloud_size=%zu, resolution=%.3f",
    cloud->size(), getResolution());

  std::vector<GridKey> updated_keys;
  updated_keys.reserve(cloud->size());

  for (const auto & pt : cloud->points)
  {
    int32_t gx, gy;
    worldToGrid(pt.x, pt.y, gx, gy);

    GridKey key{gx, gy};

    auto & cell = global_height_map_[key];

    if (!cell.has_data)
    {
      cell.min_z = pt.z;
      cell.max_z = pt.z;
      cell.mean_z = pt.z;
      cell.point_count = 1;
      cell.has_data = true;
    }
    else
    {
      if (pt.z < cell.min_z) cell.min_z = pt.z;
      if (pt.z > cell.max_z) cell.max_z = pt.z;

      float prev_mean = cell.mean_z;
      unsigned int prev_count = cell.point_count;
      cell.mean_z = (prev_mean * static_cast<float>(prev_count) + pt.z) /
                    static_cast<float>(prev_count + 1);
      cell.point_count++;
    }

    cell.last_seen_time = stamp;
    updated_keys.push_back(key);
  }

  for (const GridKey & key : updated_keys)
  {
    auto & cell = global_height_map_[key];
    cell.height_diff = cell.max_z - cell.min_z;
  }

  decayOldObservations(stamp);

  computeSlope();
}

// raycastClear is temporarily disabled.
// It needs to be rewritten to work with global_height_map_ (world coordinates)
// instead of the old rolling-window index-based height_map_.
// For now, enable_raycast_clear_ defaults to false and this function is not called.
#if 0
void TraversabilityLayer::raycastClear(
  double sensor_x, double sensor_y,
  const std::vector<std::pair<int, int>> & marked_cells)
{
  // TODO: Rewrite using global_height_map_ with world-coordinate raycasting
}
#endif

void TraversabilityLayer::decayOldObservations(const rclcpp::Time & now)
{
  for (auto it = global_height_map_.begin();
       it != global_height_map_.end();)
  {
    double age = (now - it->second.last_seen_time).seconds();

    if (age > clearing_duration_)
    {
      it = global_height_map_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void TraversabilityLayer::computeSlope()
{
  double res = getResolution();

  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  int cells_with_slope = 0;
  float max_slope_mag = 0.0f;
  float max_height_diff = 0.0f;

  for (auto & kv : global_height_map_)
  {
    const GridKey & key = kv.first;
    auto & cell = kv.second;

    if (!cell.has_data) {
      continue;
    }

    int32_t gx = key.gx;
    int32_t gy = key.gy;

    auto xp_it = global_height_map_.find(GridKey{gx + 1, gy});
    auto xm_it = global_height_map_.find(GridKey{gx - 1, gy});
    auto yp_it = global_height_map_.find(GridKey{gx, gy + 1});
    auto ym_it = global_height_map_.find(GridKey{gx, gy - 1});

    float dz_dx = 0.0f;
    float dz_dy = 0.0f;

    if (xp_it != global_height_map_.end() &&
        xm_it != global_height_map_.end())
    {
      dz_dx =
          (xp_it->second.mean_z -
           xm_it->second.mean_z) /
          (2.0f * static_cast<float>(res));
    }

    if (yp_it != global_height_map_.end() &&
        ym_it != global_height_map_.end())
    {
      dz_dy =
          (yp_it->second.mean_z -
           ym_it->second.mean_z) /
          (2.0f * static_cast<float>(res));
    }

    cell.slope_x = dz_dx;
    cell.slope_y = dz_dy;

    cell.slope_magnitude =
        std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);

    if (cell.slope_magnitude > max_slope_mag) {
      max_slope_mag = cell.slope_magnitude;
    }
    if (cell.height_diff > max_height_diff) {
      max_height_diff = cell.height_diff;
    }
    if (dz_dx != 0.0f || dz_dy != 0.0f) {
      cells_with_slope++;
    }
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] computeSlope: %d cells with slope, max_slope_mag=%.3f (angle=%.1f deg), max_height_diff=%.3f",
    cells_with_slope, max_slope_mag, std::atan(max_slope_mag) * 180.0 / M_PI, max_height_diff);
}

unsigned char TraversabilityLayer::computeCost(const CellData & cell) const
{
  if (!cell.has_data || cell.point_count < 2) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  unsigned char height_cost = 0;
  if (cell.height_diff > step_height_threshold_) {
    float excess = cell.height_diff - static_cast<float>(step_height_threshold_);
    float ratio = 1.0f - std::exp(-static_cast<float>(height_cost_scale_) * excess);
    height_cost = static_cast<unsigned char>(
      std::min(static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE),
               ratio * static_cast<float>(lethal_cost_threshold_)));
  }

  unsigned char slope_cost = 0;
  float slope_angle = std::atan(cell.slope_magnitude);

  if (slope_angle > max_slope_traversable_) {
    slope_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  } else if (slope_angle > slope_cost_start_) {
    float excess_angle = slope_angle - static_cast<float>(slope_cost_start_);
    float max_excess = static_cast<float>(max_slope_traversable_) - static_cast<float>(slope_cost_start_);
    if (max_excess > 1e-6f) {
      float ratio = excess_angle / max_excess;
      float cost_f = (1.0f - std::exp(-static_cast<float>(slope_cost_scale_) * ratio)) *
                     static_cast<float>(lethal_cost_threshold_);
      slope_cost = static_cast<unsigned char>(
        std::min(static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE), cost_f));
    }
  }

  unsigned char final_cost = std::max(height_cost, slope_cost);
  
  static int compute_debug_count = 0;
  if (final_cost == 0 && compute_debug_count < 10) {
    RCLCPP_INFO(
      rclcpp::get_logger("traversability_layer"),
      "【调试-阈值检查】cost=0: slope_angle=%.3f rad (%.1f deg), slope_cost_start=%.3f rad (%.1f deg), "
      "height_diff=%.3f, step_threshold=%.3f",
      slope_angle, slope_angle * 180.0 / M_PI,
      slope_cost_start_, slope_cost_start_ * 180.0 / M_PI,
      cell.height_diff, step_height_threshold_);
    compute_debug_count++;
  }

  return final_cost;
}

void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (global_height_map_.empty()) {
    return;
  }

  double res = getResolution();

  for (const auto & kv : global_height_map_)
  {
    const GridKey & key = kv.first;
    const auto & cell = kv.second;

    if (!cell.has_data) {
      continue;
    }

    double wx = key.gx * res;
    double wy = key.gy * res;

    *min_x = std::min(*min_x, wx);
    *min_y = std::min(*min_y, wy);

    *max_x = std::max(*max_x, wx);
    *max_y = std::max(*max_y, wy);
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
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  if (global_height_map_.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger("traversability_layer"), *clock, 5000,
      "[TraversabilityLayer] updateCosts: global_height_map_ empty");
    return;
  }

  double ox = master_grid.getOriginX();
  double oy = master_grid.getOriginY();
  double res = master_grid.getResolution();

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  unsigned char * master_array = master_grid.getCharMap();

  int cells_with_cost = 0;
  int lethal_cells = 0;

  for (const auto & kv : global_height_map_)
  {
    const GridKey & key = kv.first;
    const auto & cell = kv.second;

    if (!cell.has_data) {
      continue;
    }

    double wx = key.gx * res;
    double wy = key.gy * res;

    int mx = static_cast<int>((wx - ox) / res);
    int my = static_cast<int>((wy - oy) / res);

    if (mx < 0 || mx >= static_cast<int>(size_x) ||
        my < 0 || my >= static_cast<int>(size_y))
    {
      continue;
    }

    unsigned char cost = computeCost(cell);

    static int debug_print_count = 0;
    if (cost > 0 && debug_print_count < 20) {
      RCLCPP_INFO(
        rclcpp::get_logger("traversability_layer"),
        "【调试】成功计算出有效代价: cost=%d, slope_magnitude=%.3f, height_diff=%.3f, slope_angle=%.3f rad (%.1f deg)",
        cost, cell.slope_magnitude, cell.height_diff,
        std::atan(cell.slope_magnitude), std::atan(cell.slope_magnitude) * 180.0 / M_PI);
      debug_print_count++;
    }

    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      continue;
    }

    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      lethal_cells++;
    }
    cells_with_cost++;

    unsigned int idx = master_grid.getIndex(mx, my);
    unsigned char old_cost = master_array[idx];

    if (old_cost == nav2_costmap_2d::NO_INFORMATION ||
        cost > old_cost) {
      master_array[idx] = cost;
    }
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] updateCosts: %d cells with cost, %d lethal (range=[%d,%d,%d,%d])",
    cells_with_cost, lethal_cells, min_i, min_j, max_i, max_j);

  if (publish_slope_map_ && slope_pub_) {
    auto node = node_.lock();
    if (node) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      slope_cloud->header.frame_id = layered_costmap_->getGlobalFrameID();

      double res = getResolution();

      for (const auto & kv : global_height_map_)
      {
        const GridKey & key = kv.first;
        const auto & cell = kv.second;

        if (!cell.has_data || cell.point_count < 2) {
          continue;
        }

        pcl::PointXYZI pt;
        pt.x = static_cast<float>(key.gx * res);
        pt.y = static_cast<float>(key.gy * res);
        pt.z = cell.mean_z;
        pt.intensity = cell.slope_magnitude;
        slope_cloud->push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 slope_msg;
      pcl::toROSMsg(*slope_cloud, slope_msg);
      slope_msg.header.stamp = node->now();
      slope_pub_->publish(slope_msg);

      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("traversability_layer"), *clock, 2000,
        "[TraversabilityLayer] Published slope_map: %zu points, frame=%s",
        slope_cloud->size(), slope_cloud->header.frame_id.c_str());
    }
  } else {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger("traversability_layer"), *clock, 5000,
      "[TraversabilityLayer] slope_map not published: publish_slope_map_=%d, slope_pub_=%p",
      publish_slope_map_, slope_pub_.get());
  }
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
}

void TraversabilityLayer::resetMaps()
{
  global_height_map_.clear();
}

void TraversabilityLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  rclcpp::QoS cloud_qos(rclcpp::KeepLast(static_cast<size_t>(cloud_buffer_size_)));
  cloud_qos.best_effort();
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
