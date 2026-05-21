#include "traversability_layer/traversability_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <omp.h>

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
  declareParameter("height_cost_start", rclcpp::ParameterValue(0.0));
  declareParameter("slope_cost_scale", rclcpp::ParameterValue(5.0));
  declareParameter("height_cost_scale", rclcpp::ParameterValue(10.0));
  declareParameter("lethal_cost_threshold", rclcpp::ParameterValue(254.0));
  declareParameter("observation_persistence", rclcpp::ParameterValue(5.0));
  declareParameter("cloud_buffer_size", rclcpp::ParameterValue(5));
  declareParameter("publish_slope_map", rclcpp::ParameterValue(false));
  declareParameter("enable_raycast_clear", rclcpp::ParameterValue(false));
  declareParameter("cell_resolution", rclcpp::ParameterValue(0.0));
  declareParameter("num_threads", rclcpp::ParameterValue(0));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic_);
  node->get_parameter(name_ + ".sensor_frame", sensor_frame_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_slope_traversable", max_slope_traversable_);
  node->get_parameter(name_ + ".slope_cost_start", slope_cost_start_);
  node->get_parameter(name_ + ".step_height_threshold", step_height_threshold_);
  node->get_parameter(name_ + ".height_cost_start", height_cost_start_);
  node->get_parameter(name_ + ".slope_cost_scale", slope_cost_scale_);
  node->get_parameter(name_ + ".height_cost_scale", height_cost_scale_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(name_ + ".observation_persistence", observation_persistence_);
  node->get_parameter(name_ + ".cloud_buffer_size", cloud_buffer_size_);
  node->get_parameter(name_ + ".publish_slope_map", publish_slope_map_);
  node->get_parameter(name_ + ".enable_raycast_clear", enable_raycast_clear_);
  node->get_parameter(name_ + ".cell_resolution", cell_resolution_);
  node->get_parameter(name_ + ".num_threads", num_threads_);

  if (num_threads_ > 0) {
    omp_set_num_threads(num_threads_);
  } else if (num_threads_ == -1) {
    omp_set_num_threads(1);
  }

  last_perf_log_ = node->now();

  max_slope_traversable_ = max_slope_traversable_ * M_PI / 180.0;
  slope_cost_start_ = slope_cost_start_ * M_PI / 180.0;

  if (cell_resolution_ <= 0.0) {
    nav2_costmap_2d::Costmap2D * master_grid = layered_costmap_->getCostmap();
    cell_resolution_ = master_grid->getResolution();
  }

  RCLCPP_INFO(
    node->get_logger(),
    "TraversabilityLayer: step_height_threshold=%.3f, max_slope_traversable=%.1f deg, "
    "slope_cost_start=%.1f deg, pointcloud_topic=%s, "
    "enable_raycast_clear=%s, cell_resolution=%.3f, num_threads=%d",
    step_height_threshold_, max_slope_traversable_ * 180.0 / M_PI,
    slope_cost_start_ * 180.0 / M_PI, pointcloud_topic_.c_str(),
    enable_raycast_clear_ ? "true" : "false",
    cell_resolution_, num_threads_);

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

  matchSize();

  current_ = true;
}

void TraversabilityLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master_grid = layered_costmap_->getCostmap();

  resizeMap(master_grid->getSizeInCellsX(), master_grid->getSizeInCellsY(),
            master_grid->getResolution(),
            master_grid->getOriginX(), master_grid->getOriginY());

  double world_w = master_grid->getSizeInCellsX() * master_grid->getResolution();
  double world_h = master_grid->getSizeInCellsY() * master_grid->getResolution();

  grid_size_x_ = static_cast<unsigned int>(std::ceil(world_w / cell_resolution_));
  grid_size_y_ = static_cast<unsigned int>(std::ceil(world_h / cell_resolution_));
  grid_map_.assign(grid_size_x_ * grid_size_y_, CellData{});
}

void TraversabilityLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  RCLCPP_DEBUG_THROTTLE(
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

  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] TF lookup: target=%s, source=%s",
    target_frame.c_str(), source_frame.c_str());

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
    RCLCPP_DEBUG_THROTTLE(
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

  sensor_global_x_ = tx;
  sensor_global_y_ = ty;
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

  std::vector<Point3D> transformed_cloud;
  transformed_cloud.reserve(cloud->size());

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

    transformed_cloud.push_back({x_global, y_global, z_global});
  }

  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] After transform: %zu points kept, %d filtered (z_range=[%.2f, %.2f])",
    transformed_cloud.size(), filtered_count, min_obstacle_height_, max_obstacle_height_);

  if (observation_persistence_ <= 0.0) {
    accumulated_cloud_.clear();
  }
  rclcpp::Time arrival_time = clock->now();
  for (const auto & pt : transformed_cloud) {
    accumulated_cloud_.push_back({pt, arrival_time});
  }
  const size_t max_points = 200000;
  if (accumulated_cloud_.size() > max_points) {
    accumulated_cloud_.erase(accumulated_cloud_.begin(),
      accumulated_cloud_.begin() + static_cast<long>(accumulated_cloud_.size() - max_points));
  }
  cloud_updated_ = true;
}

void TraversabilityLayer::computeSlope(double origin_x, double origin_y)
{
  static auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  int cells_with_slope = 0;
  float max_slope_mag = 0.0f;
  float max_height_diff = 0.0f;

  float sensor_gx = static_cast<float>((sensor_global_x_ - origin_x) / cell_resolution_);
  float sensor_gy = static_cast<float>((sensor_global_y_ - origin_y) / cell_resolution_);

#pragma omp parallel for collapse(2) reduction(+:cells_with_slope) reduction(max:max_slope_mag) reduction(max:max_height_diff)
  for (int cy = 0; cy < static_cast<int>(grid_size_y_); cy++) {
    for (int cx = 0; cx < static_cast<int>(grid_size_x_); cx++) {
      size_t idx = gridIndex(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy));
      auto & cell = grid_map_[idx];

      if (!cell.has_data) {
        continue;
      }

      cell.slope_magnitude = 0.0f;

      float dx = sensor_gx - static_cast<float>(cx);
      float dy = sensor_gy - static_cast<float>(cy);
      float dist = std::sqrt(dx * dx + dy * dy);

      if (dist < 1.0f) {
        continue;
      }

      int n_steps = static_cast<int>(std::ceil(dist / 0.5f));
      float step_x = dx / static_cast<float>(n_steps);
      float step_y = dy / static_cast<float>(n_steps);

      float cur_x = static_cast<float>(cx) + 0.5f;
      float cur_y = static_cast<float>(cy) + 0.5f;

      float ref_z = cell.representative_z;
      float ref_dist = 0.0f;
      bool found_ref = false;

      for (int i = 1; i <= n_steps; i++) {
        cur_x += step_x;
        cur_y += step_y;

        int check_cx = static_cast<int>(std::floor(cur_x));
        int check_cy = static_cast<int>(std::floor(cur_y));

        if (check_cx == static_cast<int>(cx) && check_cy == static_cast<int>(cy)) {
          continue;
        }

        if (check_cx < 0 || check_cx >= static_cast<int>(grid_size_x_) ||
            check_cy < 0 || check_cy >= static_cast<int>(grid_size_y_)) {
          break;
        }

        size_t ref_idx = gridIndex(
          static_cast<unsigned int>(check_cx), static_cast<unsigned int>(check_cy));
        const auto & ref_cell = grid_map_[ref_idx];

        if (ref_cell.has_data) {
          ref_z = ref_cell.representative_z;
          float ddx = cur_x - (static_cast<float>(cx) + 0.5f);
          float ddy = cur_y - (static_cast<float>(cy) + 0.5f);
          ref_dist = std::sqrt(ddx * ddx + ddy * ddy) * static_cast<float>(cell_resolution_);
          found_ref = true;
          break;
        }
      }

      if (found_ref && ref_dist > 1e-6f) {
        float dz = std::abs(cell.representative_z - ref_z);

        cell.height_diff = dz;
        cell.slope_magnitude = dz / ref_dist;

        if (cell.slope_magnitude > max_slope_mag) {
          max_slope_mag = cell.slope_magnitude;
        }
        if (dz > max_height_diff) {
          max_height_diff = dz;
        }
        if (cell.slope_magnitude > 0.0f) {
          cells_with_slope++;
        }
      }
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] computeSlope: %d cells with slope, max_slope_mag=%.3f (angle=%.1f deg), max_height_diff=%.3f",
    cells_with_slope, max_slope_mag, std::atan(max_slope_mag) * 180.0 / M_PI, max_height_diff);
}

unsigned char TraversabilityLayer::computeCost(const CellData & cell) const
{
  if (!cell.has_data) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  float height_above_ground = cell.height_diff;

  if (height_above_ground <= static_cast<float>(height_cost_start_)) {
    return 0;
  }

  float range = static_cast<float>(step_height_threshold_) - static_cast<float>(height_cost_start_);
  float norm_height = height_above_ground - static_cast<float>(height_cost_start_);

  if (range > 1e-6f) {
    float ratio = norm_height / range;
    if (ratio > 1.0f) {
      ratio = 1.0f;
    }
    float cost_f = 1.0f + ratio * 252.0f;
    unsigned char height_cost = static_cast<unsigned char>(std::min(253.0f, cost_f));

    if (height_above_ground <= static_cast<float>(step_height_threshold_)) {
      return height_cost;
    }

    float slope_angle = std::atan(cell.slope_magnitude);

    if (slope_angle > max_slope_traversable_) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    return height_cost;
  }

  return static_cast<unsigned char>(std::min(253.0f, (height_above_ground / static_cast<float>(step_height_threshold_)) * 253.0f));
}

void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  nav2_costmap_2d::Costmap2D * master_grid = layered_costmap_->getCostmap();
  double ox = master_grid->getOriginX();
  double oy = master_grid->getOriginY();
  double res = master_grid->getResolution();
  unsigned int sx = master_grid->getSizeInCellsX();
  unsigned int sy = master_grid->getSizeInCellsY();

  *min_x = std::min(*min_x, ox);
  *min_y = std::min(*min_y, oy);
  *max_x = std::max(*max_x, ox + sx * res);
  *max_y = std::max(*max_y, oy + sy * res);
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
  rclcpp::Time frame_start = clock->now();

  double ox = master_grid.getOriginX();
  double oy = master_grid.getOriginY();
  double costmap_res = master_grid.getResolution();
  unsigned int costmap_sx = master_grid.getSizeInCellsX();
  unsigned int costmap_sy = master_grid.getSizeInCellsY();

  double world_w = costmap_sx * costmap_res;
  double world_h = costmap_sy * costmap_res;
  unsigned int new_gx = static_cast<unsigned int>(std::ceil(world_w / cell_resolution_));
  unsigned int new_gy = static_cast<unsigned int>(std::ceil(world_h / cell_resolution_));

  if (new_gx != grid_size_x_ || new_gy != grid_size_y_) {
    grid_size_x_ = new_gx;
    grid_size_y_ = new_gy;
    grid_map_.resize(grid_size_x_ * grid_size_y_);
  }

  grid_map_.assign(grid_size_x_ * grid_size_y_, CellData{});

  if (accumulated_cloud_.empty()) {
    return;
  }

  if (observation_persistence_ > 0.0) {
    auto node = node_.lock();
    if (node) {
      rclcpp::Time now = node->now();
      auto cutoff = now - rclcpp::Duration::from_seconds(observation_persistence_);
      accumulated_cloud_.erase(
        std::remove_if(accumulated_cloud_.begin(), accumulated_cloud_.end(),
          [&](const TimedPoint & tp) { return tp.stamp < cutoff; }),
        accumulated_cloud_.end());
    }
  }

  double inv_cell_res = 1.0 / cell_resolution_;

#pragma omp parallel for
  for (int pi = 0; pi < static_cast<int>(accumulated_cloud_.size()); pi++)
  {
    const auto & pt = accumulated_cloud_[static_cast<size_t>(pi)].pt;
    int cx = static_cast<int>(std::floor((pt.x - ox) * inv_cell_res));
    int cy = static_cast<int>(std::floor((pt.y - oy) * inv_cell_res));

    if (cx < 0 || cx >= static_cast<int>(grid_size_x_) ||
        cy < 0 || cy >= static_cast<int>(grid_size_y_))
    {
      continue;
    }

    size_t idx = gridIndex(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy));
    auto & cell = grid_map_[idx];

    float z = static_cast<float>(pt.z);

#pragma omp critical(grid_update)
    {
      if (!cell.has_data)
      {
        cell.min_z = z;
        cell.max_z = z;
        cell.representative_z = z;
        cell.point_count = 1;
        cell.has_data = true;
      }
      else
      {
        if (z < cell.min_z) cell.min_z = z;
        if (z > cell.max_z) cell.max_z = z;
        cell.point_count++;
      }
    }
  }

  for (auto & cell : grid_map_)
  {
    if (cell.has_data) {
      cell.representative_z = cell.max_z;
    }
  }

  computeSlope(ox, oy);

  unsigned char * master_array = master_grid.getCharMap();
  int cells_with_cost = 0;
  int lethal_cells = 0;
  double inv_costmap_res = 1.0 / costmap_res;

#pragma omp parallel for collapse(2) reduction(+:cells_with_cost) reduction(+:lethal_cells)
  for (unsigned int cy = 0; cy < grid_size_y_; cy++) {
    for (unsigned int cx = 0; cx < grid_size_x_; cx++) {
      size_t idx = gridIndex(cx, cy);
      const auto & cell = grid_map_[idx];

      if (!cell.has_data) {
        continue;
      }

      unsigned char cost = computeCost(cell);

      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        lethal_cells++;
      }
      cells_with_cost++;

      double cell_wx = cx * cell_resolution_ + ox;
      double cell_wy = cy * cell_resolution_ + oy;

      int mx_start = static_cast<int>(std::floor((cell_wx - ox) * inv_costmap_res));
      int my_start = static_cast<int>(std::floor((cell_wy - oy) * inv_costmap_res));
      int mx_end = static_cast<int>(std::floor((cell_wx + cell_resolution_ - ox - 1e-6) * inv_costmap_res));
      int my_end = static_cast<int>(std::floor((cell_wy + cell_resolution_ - oy - 1e-6) * inv_costmap_res));

      mx_end = std::min(mx_end, static_cast<int>(costmap_sx) - 1);
      my_end = std::min(my_end, static_cast<int>(costmap_sy) - 1);

      for (int my = my_start; my <= my_end; my++) {
        for (int mx = mx_start; mx <= mx_end; mx++) {
          if (mx < 0 || my < 0) continue;

          unsigned int master_idx = master_grid.getIndex(
            static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
#pragma omp critical(cost_write)
          {
            unsigned char old_cost = master_array[master_idx];
            if (old_cost == nav2_costmap_2d::NO_INFORMATION || cost > old_cost) {
              master_array[master_idx] = cost;
            }
          }
        }
      }
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] updateCosts: %d cells with cost, %d lethal (range=[%d,%d,%d,%d])",
    cells_with_cost, lethal_cells, min_i, min_j, max_i, max_j);

  rclcpp::Time frame_end = clock->now();
  double frame_ms = (frame_end - frame_start).seconds() * 1000.0;
  perf_total_time_ += frame_ms;
  perf_frame_count_++;

  double elapsed = (frame_end - last_perf_log_).seconds();
  if (elapsed >= 10.0) {
    double avg_ms = perf_total_time_ / static_cast<double>(perf_frame_count_);
    RCLCPP_INFO(
      rclcpp::get_logger("traversability_layer"),
      "[TraversabilityLayer] Perf: %d frames in %.1fs, avg=%.2fms, last=%.2fms, "
      "cells_with_cost=%d, lethal=%d, cloud_points=%zu",
      perf_frame_count_, elapsed, avg_ms, frame_ms,
      cells_with_cost, lethal_cells,
      accumulated_cloud_.size());
    perf_frame_count_ = 0;
    perf_total_time_ = 0.0;
    last_perf_log_ = frame_end;
  }

  if (publish_slope_map_ && slope_pub_) {
    auto node = node_.lock();
    if (node) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      slope_cloud->header.frame_id = layered_costmap_->getGlobalFrameID();

      for (unsigned int cy = 0; cy < grid_size_y_; cy++) {
        for (unsigned int cx = 0; cx < grid_size_x_; cx++) {
          size_t idx = gridIndex(cx, cy);
          const auto & cell = grid_map_[idx];

          if (!cell.has_data) {
            continue;
          }

          pcl::PointXYZI pt;
          pt.x = static_cast<float>(cx * cell_resolution_ + ox);
          pt.y = static_cast<float>(cy * cell_resolution_ + oy);
          pt.z = cell.representative_z;
          pt.intensity = cell.slope_magnitude;
          slope_cloud->push_back(pt);
        }
      }

      sensor_msgs::msg::PointCloud2 slope_msg;
      pcl::toROSMsg(*slope_cloud, slope_msg);
      slope_msg.header.stamp = node->now();
      slope_pub_->publish(slope_msg);

      RCLCPP_DEBUG_THROTTLE(
        rclcpp::get_logger("traversability_layer"), *clock, 2000,
        "[TraversabilityLayer] Published slope_map: %zu points, frame=%s",
        slope_cloud->size(), slope_cloud->header.frame_id.c_str());
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
  grid_map_.assign(grid_size_x_ * grid_size_y_, CellData{});
  accumulated_cloud_.clear();
  cloud_updated_ = false;
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
