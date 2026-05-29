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
  declareParameter("cell_resolution", rclcpp::ParameterValue(0.0));
  declareParameter("num_threads", rclcpp::ParameterValue(0));
  declareParameter("voxel_z_resolution", rclcpp::ParameterValue(0.1));
  declareParameter("voxel_z_min", rclcpp::ParameterValue(-1.0));
  declareParameter("voxel_z_max", rclcpp::ParameterValue(3.0));
  declareParameter("ground_hit_threshold", rclcpp::ParameterValue(1));
  declareParameter("free_space_threshold", rclcpp::ParameterValue(1));
  declareParameter("free_space_window", rclcpp::ParameterValue(3));
  declareParameter("interp_search_radius", rclcpp::ParameterValue(3));
  declareParameter("min_interp_neighbors", rclcpp::ParameterValue(2));
  declareParameter("robot_height", rclcpp::ParameterValue(0.5));
  declareParameter("obstacle_ratio_threshold", rclcpp::ParameterValue(0.5));

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
  node->get_parameter(name_ + ".cell_resolution", cell_resolution_);
  node->get_parameter(name_ + ".num_threads", num_threads_);
  node->get_parameter(name_ + ".voxel_z_resolution", voxel_z_resolution_);
  node->get_parameter(name_ + ".voxel_z_min", voxel_z_min_);
  node->get_parameter(name_ + ".voxel_z_max", voxel_z_max_);
  node->get_parameter(name_ + ".ground_hit_threshold", ground_hit_threshold_);
  node->get_parameter(name_ + ".free_space_threshold", free_space_threshold_);
  node->get_parameter(name_ + ".free_space_window", free_space_window_);
  node->get_parameter(name_ + ".interp_search_radius", interp_search_radius_);
  node->get_parameter(name_ + ".min_interp_neighbors", min_interp_neighbors_);
  node->get_parameter(name_ + ".robot_height", robot_height_);
  node->get_parameter(name_ + ".obstacle_ratio_threshold", obstacle_ratio_threshold_);

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
    "TraversabilityLayer(v3d): step_height=%.3f, max_slope=%.1fdeg, slope_start=%.1fdeg, "
    "topic=%s, cell_res=%.3f, voxel_z_res=%.3f, z_range=[%.1f,%.1f], "
    "ground_hit_thr=%d, free_space_thr=%d, free_space_win=%d, "
    "interp_radius=%d, min_interp=%d, robot_height=%.2f, obstacle_ratio_thr=%.2f, num_threads=%d",
    step_height_threshold_, max_slope_traversable_ * 180.0 / M_PI,
    slope_cost_start_ * 180.0 / M_PI, pointcloud_topic_.c_str(),
    cell_resolution_, voxel_z_resolution_, voxel_z_min_, voxel_z_max_,
    ground_hit_threshold_, free_space_threshold_, free_space_window_,
    interp_search_radius_, min_interp_neighbors_,
    robot_height_, obstacle_ratio_threshold_, num_threads_);

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

  ground_size_x_ = static_cast<unsigned int>(std::ceil(world_w / cell_resolution_));
  ground_size_y_ = static_cast<unsigned int>(std::ceil(world_h / cell_resolution_));
  ground_map_.assign(ground_size_x_ * ground_size_y_, GroundCell{});
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

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("traversability_layer"), *clock, 2000,
      "[TraversabilityLayer] TF failed: %s (target=%s, source=%s)",
      ex.what(), target_frame.c_str(), source_frame.c_str());
    return;
  }

  double tx = transform.transform.translation.x;
  double ty = transform.transform.translation.y;
  double tz = transform.transform.translation.z;

  sensor_global_x_ = tx;
  sensor_global_y_ = ty;
  sensor_global_z_ = tz;

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

  Point3D sensor_pos{tx, ty, tz};
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
    "[TraversabilityLayer] After transform: %zu kept, %d filtered",
    transformed_cloud.size(), filtered_count);

  nav2_costmap_2d::Costmap2D * master_grid = layered_costmap_->getCostmap();
  double ox = master_grid->getOriginX();
  double oy = master_grid->getOriginY();

  incrementalUpdateVoxelGrid(transformed_cloud, sensor_pos, ox, oy);
  cloud_updated_ = true;
}

void TraversabilityLayer::shiftVoxelGrid(int shift_x, int shift_y)
{
  if (shift_x == 0 && shift_y == 0) return;

  std::vector<VoxelData> new_grid(
    static_cast<size_t>(voxel_size_x_) *
    static_cast<size_t>(voxel_size_y_) *
    static_cast<size_t>(voxel_size_z_), VoxelData{});

#pragma omp parallel for collapse(2) schedule(static)
  for (int ny = 0; ny < static_cast<int>(voxel_size_y_); ny++) {
    for (int nx = 0; nx < static_cast<int>(voxel_size_x_); nx++) {
      int old_x = nx - shift_x;
      int old_y = ny - shift_y;

      if (old_x < 0 || old_x >= static_cast<int>(voxel_size_x_) ||
          old_y < 0 || old_y >= static_cast<int>(voxel_size_y_))
      {
        continue;
      }

      for (unsigned int iz = 0; iz < voxel_size_z_; iz++) {
        size_t new_idx = voxelIndex(
          static_cast<unsigned int>(nx),
          static_cast<unsigned int>(ny), iz);
        size_t old_idx = voxelIndex(
          static_cast<unsigned int>(old_x),
          static_cast<unsigned int>(old_y), iz);
        new_grid[new_idx] = voxel_grid_[old_idx];
      }
    }
  }

  voxel_grid_ = std::move(new_grid);
  voxel_ox_ -= shift_x * cell_resolution_;
  voxel_oy_ -= shift_y * cell_resolution_;
}

void TraversabilityLayer::expandVoxelGridZ(double new_z_lo, double new_z_hi)
{
  unsigned int new_size_z = static_cast<unsigned int>(
    std::ceil((new_z_hi - new_z_lo) / voxel_z_resolution_));
  if (new_size_z < 1) new_size_z = 1;

  if (new_size_z == voxel_size_z_ &&
      std::abs(new_z_lo - voxel_z_origin_) < voxel_z_resolution_ * 0.5)
  {
    return;
  }

  int z_offset = static_cast<int>(
    std::round((voxel_z_origin_ - new_z_lo) / voxel_z_resolution_));

  std::vector<VoxelData> new_grid(
    static_cast<size_t>(voxel_size_x_) *
    static_cast<size_t>(voxel_size_y_) *
    static_cast<size_t>(new_size_z), VoxelData{});

#pragma omp parallel for collapse(2) schedule(static)
  for (int y = 0; y < static_cast<int>(voxel_size_y_); y++) {
    for (int x = 0; x < static_cast<int>(voxel_size_x_); x++) {
      for (unsigned int old_iz = 0; old_iz < voxel_size_z_; old_iz++) {
        int new_iz = static_cast<int>(old_iz) + z_offset;
        if (new_iz < 0 || new_iz >= static_cast<int>(new_size_z)) continue;

        size_t old_idx = voxelIndex(
          static_cast<unsigned int>(x),
          static_cast<unsigned int>(y), old_iz);
        size_t new_idx = static_cast<size_t>(new_iz) *
          static_cast<size_t>(voxel_size_x_) *
          static_cast<size_t>(voxel_size_y_) +
          static_cast<size_t>(y) * static_cast<size_t>(voxel_size_x_) +
          static_cast<size_t>(x);
        new_grid[new_idx] = voxel_grid_[old_idx];
      }
    }
  }

  voxel_grid_ = std::move(new_grid);
  voxel_z_origin_ = new_z_lo;
  voxel_size_z_ = new_size_z;
}

void TraversabilityLayer::incrementalUpdateVoxelGrid(
  const std::vector<Point3D> & transformed_pts,
  const Point3D & sensor_pos, double ox, double oy)
{
  unsigned int new_size_x = ground_size_x_;
  unsigned int new_size_y = ground_size_y_;

  double z_min_world = std::numeric_limits<double>::max();
  double z_max_world = std::numeric_limits<double>::lowest();
  double cur_sensor_z = sensor_global_z_;

#pragma omp parallel for reduction(min:z_min_world) reduction(max:z_max_world)
  for (int i = 0; i < static_cast<int>(transformed_pts.size()); i++) {
    if (transformed_pts[i].z < z_min_world) z_min_world = transformed_pts[i].z;
    if (transformed_pts[i].z > z_max_world) z_max_world = transformed_pts[i].z;
  }
  if (sensor_pos.z < z_min_world) z_min_world = sensor_pos.z;
  if (sensor_pos.z > z_max_world) z_max_world = sensor_pos.z;

  double z_lo = std::min(z_min_world, cur_sensor_z + voxel_z_min_) - voxel_z_resolution_;
  double z_hi = std::max(z_max_world, cur_sensor_z + voxel_z_max_) + voxel_z_resolution_;

  if (!voxel_grid_valid_ ||
      new_size_x != voxel_size_x_ || new_size_y != voxel_size_y_) {
    voxel_size_x_ = new_size_x;
    voxel_size_y_ = new_size_y;
    voxel_z_origin_ = z_lo;
    voxel_ox_ = ox;
    voxel_oy_ = oy;
    voxel_size_z_ = static_cast<unsigned int>(
      std::ceil((z_hi - z_lo) / voxel_z_resolution_));
    if (voxel_size_z_ < 1) voxel_size_z_ = 1;

    size_t total_voxels = static_cast<size_t>(voxel_size_x_) *
                          static_cast<size_t>(voxel_size_y_) *
                          static_cast<size_t>(voxel_size_z_);
    voxel_grid_.assign(total_voxels, VoxelData{});
    voxel_grid_valid_ = true;
  } else {
    int shift_x = static_cast<int>(std::round((voxel_ox_ - ox) / cell_resolution_));
    int shift_y = static_cast<int>(std::round((voxel_oy_ - oy) / cell_resolution_));
    shiftVoxelGrid(shift_x, shift_y);

    voxel_ox_ = ox;
    voxel_oy_ = oy;

    bool z_expand = false;
    if (z_lo < voxel_z_origin_ - voxel_z_resolution_ * 0.5) z_expand = true;
    if (z_hi > voxel_z_origin_ + voxel_size_z_ * voxel_z_resolution_ + voxel_z_resolution_ * 0.5) z_expand = true;

    if (z_expand) {
      double expanded_z_lo = std::min(z_lo, voxel_z_origin_);
      double expanded_z_hi = std::max(z_hi, voxel_z_origin_ + voxel_size_z_ * voxel_z_resolution_);
      expandVoxelGridZ(expanded_z_lo, expanded_z_hi);
    }
  }

  frame_counter_++;

  double inv_cell_res = 1.0 / cell_resolution_;
  double inv_vz_res = 1.0 / voxel_z_resolution_;
  double vox_ox = voxel_ox_;
  double vox_oy = voxel_oy_;

  int n_threads = omp_get_max_threads();
  struct ThreadLocalHit { size_t idx; bool is_hit; };
  std::vector<std::vector<ThreadLocalHit>> thread_buffers(n_threads);

  for (int t = 0; t < n_threads; t++) {
    thread_buffers[t].reserve(512);
  }

#pragma omp parallel
  {
    int tid = omp_get_thread_num();
    auto & local_buf = thread_buffers[tid];

#pragma omp for schedule(dynamic, 512) nowait
    for (int pi = 0; pi < static_cast<int>(transformed_pts.size()); pi++)
    {
      const auto & pt = transformed_pts[pi];
      const auto & sp = sensor_pos;

      int ix = static_cast<int>(std::floor((pt.x - vox_ox) * inv_cell_res));
      int iy = static_cast<int>(std::floor((pt.y - vox_oy) * inv_cell_res));
      int iz = static_cast<int>(std::floor((pt.z - voxel_z_origin_) * inv_vz_res));

      if (ix < 0 || ix >= static_cast<int>(voxel_size_x_) ||
          iy < 0 || iy >= static_cast<int>(voxel_size_y_) ||
          iz < 0 || iz >= static_cast<int>(voxel_size_z_))
      {
        continue;
      }

      size_t hit_idx = voxelIndex(
        static_cast<unsigned int>(ix),
        static_cast<unsigned int>(iy),
        static_cast<unsigned int>(iz));

      local_buf.push_back({hit_idx, true});

      int six = static_cast<int>(std::floor((sp.x - vox_ox) * inv_cell_res));
      int siy = static_cast<int>(std::floor((sp.y - vox_oy) * inv_cell_res));
      int siz = static_cast<int>(std::floor((sp.z - voxel_z_origin_) * inv_vz_res));

      six = std::max(0, std::min(six, static_cast<int>(voxel_size_x_) - 1));
      siy = std::max(0, std::min(siy, static_cast<int>(voxel_size_y_) - 1));
      siz = std::max(0, std::min(siz, static_cast<int>(voxel_size_z_) - 1));

      double dx = pt.x - sp.x;
      double dy = pt.y - sp.y;
      double dz = pt.z - sp.z;
      double ray_len = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (ray_len < 1e-6) continue;

      double min_step = std::min(cell_resolution_, voxel_z_resolution_);
      int n_steps = static_cast<int>(std::ceil(ray_len / (min_step * 0.75)));
      double step_dx = dx / static_cast<double>(n_steps);
      double step_dy = dy / static_cast<double>(n_steps);
      double step_dz = dz / static_cast<double>(n_steps);

      int prev_ix = six, prev_iy = siy, prev_iz = siz;

      for (int s = 1; s < n_steps; s++) {
        double cx = sp.x + s * step_dx;
        double cy = sp.y + s * step_dy;
        double cz = sp.z + s * step_dz;

        int cix = static_cast<int>(std::floor((cx - vox_ox) * inv_cell_res));
        int ciy = static_cast<int>(std::floor((cy - vox_oy) * inv_cell_res));
        int ciz = static_cast<int>(std::floor((cz - voxel_z_origin_) * inv_vz_res));

        if (cix < 0 || cix >= static_cast<int>(voxel_size_x_) ||
            ciy < 0 || ciy >= static_cast<int>(voxel_size_y_) ||
            ciz < 0 || ciz >= static_cast<int>(voxel_size_z_))
        {
          continue;
        }

        if (cix == prev_ix && ciy == prev_iy && ciz == prev_iz) {
          continue;
        }

        prev_ix = cix;
        prev_iy = ciy;
        prev_iz = ciz;

        if (cix == ix && ciy == iy && ciz == iz) {
          break;
        }

        size_t pass_idx = voxelIndex(
          static_cast<unsigned int>(cix),
          static_cast<unsigned int>(ciy),
          static_cast<unsigned int>(ciz));

        local_buf.push_back({pass_idx, false});
      }
    }

#pragma omp barrier

#pragma omp for schedule(static)
    for (int t = 0; t < n_threads; t++) {
      for (const auto & entry : thread_buffers[t]) {
        if (entry.is_hit) {
          auto & v = voxel_grid_[entry.idx];
          if (v.hit_count < 255) v.hit_count++;
          v.last_update_frame = frame_counter_;
        } else {
          auto & v = voxel_grid_[entry.idx];
          if (v.pass_count < 255) v.pass_count++;
          v.last_update_frame = frame_counter_;
        }
      }
    }
  }
}

void TraversabilityLayer::decayVoxelGrid()
{
  if (observation_persistence_ <= 0.0) {
    voxel_grid_.assign(voxel_grid_.size(), VoxelData{});
    return;
  }

  uint16_t decay_frames = static_cast<uint16_t>(
    observation_persistence_ * decay_interval_frames_);
  if (decay_frames == 0) decay_frames = 1;

#pragma omp parallel for schedule(static)
  for (int i = 0; i < static_cast<int>(voxel_grid_.size()); i++) {
    auto & v = voxel_grid_[i];
    if (v.hit_count == 0 && v.pass_count == 0) continue;

    uint16_t age = frame_counter_ - v.last_update_frame;
    if (age > decay_frames) {
      v.hit_count = 0;
      v.pass_count = 0;
      v.last_update_frame = 0;
    } else if (age > decay_frames / 2) {
      v.hit_count = v.hit_count >> 1;
      v.pass_count = v.pass_count >> 1;
    }
  }
}

void TraversabilityLayer::extractGround(double ox, double oy)
{
  ground_map_.assign(ground_size_x_ * ground_size_y_, GroundCell{});

  double inv_cell_res = 1.0 / cell_resolution_;
  double inv_vz_res = 1.0 / voxel_z_resolution_;
  double sensor_z = sensor_global_z_;
  int win = free_space_window_;

  int vox_offset_x = static_cast<int>(std::round((voxel_ox_ - ox) * inv_cell_res));
  int vox_offset_y = static_cast<int>(std::round((voxel_oy_ - oy) * inv_cell_res));

  int ground_found = 0;
  int total_scanned = 0;

#pragma omp parallel for collapse(2) schedule(static) reduction(+:ground_found) reduction(+:total_scanned)
  for (int cy = 0; cy < static_cast<int>(ground_size_y_); cy++) {
    for (int cx = 0; cx < static_cast<int>(ground_size_x_); cx++) {
      int vix = cx + vox_offset_x;
      int viy = cy + vox_offset_y;

      if (vix < 0 || vix >= static_cast<int>(voxel_size_x_) ||
          viy < 0 || viy >= static_cast<int>(voxel_size_y_))
      {
        continue;
      }
      total_scanned++;

      unsigned int uix = static_cast<unsigned int>(vix);
      unsigned int uiy = static_cast<unsigned int>(viy);

      int ground_iz = -1;
      float ground_z_val = 0.0f;

      for (unsigned int iz = 0; iz < voxel_size_z_; iz++) {
        size_t vidx = voxelIndex(uix, uiy, iz);
        const auto & voxel = voxel_grid_[vidx];

        if (voxel.hit_count < static_cast<uint16_t>(ground_hit_threshold_)) {
          continue;
        }

        double voxel_world_z = voxel_z_origin_ + (iz + 0.5) * voxel_z_resolution_;

        if (voxel_world_z > sensor_z) {
          break;
        }

        bool has_free_above = false;
        for (int w = 1; w <= win; w++) {
          unsigned int wiz = iz + static_cast<unsigned int>(w);
          if (wiz >= voxel_size_z_) break;
          size_t widx = voxelIndex(uix, uiy, wiz);
          if (voxel_grid_[widx].pass_count >= static_cast<uint16_t>(free_space_threshold_)) {
            has_free_above = true;
            break;
          }
        }

        if (has_free_above) {
          ground_iz = static_cast<int>(iz);
          ground_z_val = static_cast<float>(voxel_world_z);
          break;
        }
      }

      if (ground_iz < 0) {
        for (unsigned int iz = 0; iz < voxel_size_z_; iz++) {
          size_t vidx = voxelIndex(uix, uiy, iz);
          const auto & voxel = voxel_grid_[vidx];

          if (voxel.hit_count < static_cast<uint16_t>(ground_hit_threshold_)) {
            continue;
          }

          double voxel_world_z = voxel_z_origin_ + (iz + 0.5) * voxel_z_resolution_;
          if (voxel_world_z > sensor_z) {
            break;
          }

          ground_iz = static_cast<int>(iz);
          ground_z_val = static_cast<float>(voxel_world_z);
          break;
        }
      }

      if (ground_iz >= 0) {
        size_t gidx = groundIndex(static_cast<unsigned int>(cx),
                                  static_cast<unsigned int>(cy));
        ground_map_[gidx].ground_z = ground_z_val;
        ground_map_[gidx].has_ground = true;
        ground_found++;

        double obs_z_start = ground_z_val + voxel_z_resolution_;
        double obs_z_end = ground_z_val + static_cast<double>(robot_height_);
        unsigned int iz_start = static_cast<unsigned int>(
          std::floor((obs_z_start - voxel_z_origin_) * inv_vz_res));
        unsigned int iz_end = static_cast<unsigned int>(
          std::ceil((obs_z_end - voxel_z_origin_) * inv_vz_res));
        iz_start = std::max(iz_start, static_cast<unsigned int>(ground_iz) + 1);
        iz_end = std::min(iz_end, voxel_size_z_);

        int obs_layers = 0;
        int total_layers = 0;
        for (unsigned int oiz = iz_start; oiz < iz_end; oiz++) {
          size_t oidx = voxelIndex(uix, uiy, oiz);
          total_layers++;
          if (voxel_grid_[oidx].hit_count >= static_cast<uint8_t>(ground_hit_threshold_)) {
            obs_layers++;
          }
        }

        if (total_layers > 0) {
          ground_map_[gidx].obstacle_ratio =
            static_cast<float>(obs_layers) / static_cast<float>(total_layers);
        }
      }
    }
  }

  static auto extract_ground_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"),
    *extract_ground_clock, 2000,
    "[TraversabilityLayer] extractGround: scanned=%d, ground_found=%d, "
    "vox_offset=(%d,%d), voxel_ox=%.3f, voxel_oy=%.3f, ox=%.3f, oy=%.3f",
    total_scanned, ground_found, vox_offset_x, vox_offset_y,
    voxel_ox_, voxel_oy_, ox, oy);
}

void TraversabilityLayer::interpolateGround()
{
  std::vector<GroundCell> interpolated = ground_map_;

  int radius = interp_search_radius_;
  int min_neighbors = min_interp_neighbors_;

#pragma omp parallel for collapse(2) schedule(static)
  for (int cy = 0; cy < static_cast<int>(ground_size_y_); cy++) {
    for (int cx = 0; cx < static_cast<int>(ground_size_x_); cx++) {
      size_t idx = groundIndex(static_cast<unsigned int>(cx),
                               static_cast<unsigned int>(cy));
      if (ground_map_[idx].has_ground) {
        continue;
      }

      float sum_w = 0.0f;
      float sum_wz = 0.0f;
      int neighbor_count = 0;

      for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
          if (dx == 0 && dy == 0) continue;

          int nx = cx + dx;
          int ny = cy + dy;
          if (nx < 0 || nx >= static_cast<int>(ground_size_x_) ||
              ny < 0 || ny >= static_cast<int>(ground_size_y_))
          {
            continue;
          }

          size_t nidx = groundIndex(static_cast<unsigned int>(nx),
                                    static_cast<unsigned int>(ny));
          if (!ground_map_[nidx].has_ground) continue;

          float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
          float w = 1.0f / (dist * dist);
          sum_w += w;
          sum_wz += w * ground_map_[nidx].ground_z;
          neighbor_count++;
        }
      }

      if (neighbor_count >= min_neighbors && sum_w > 1e-8f) {
        interpolated[idx].ground_z = sum_wz / sum_w;
        interpolated[idx].has_ground = true;
      }
    }
  }

  ground_map_ = std::move(interpolated);
}

void TraversabilityLayer::computeGroundSlope()
{
  float res = static_cast<float>(cell_resolution_);

#pragma omp parallel for collapse(2) schedule(static)
  for (int cy = 0; cy < static_cast<int>(ground_size_y_); cy++) {
    for (int cx = 0; cx < static_cast<int>(ground_size_x_); cx++) {
      size_t idx = groundIndex(static_cast<unsigned int>(cx),
                               static_cast<unsigned int>(cy));
      if (!ground_map_[idx].has_ground) continue;

      float z_center = ground_map_[idx].ground_z;
      float max_diff = 0.0f;
      float slope_x = 0.0f;
      float slope_y = 0.0f;

      float z_xp = 0.0f, z_xm = 0.0f, z_yp = 0.0f, z_ym = 0.0f;
      bool has_xp = false, has_xm = false, has_yp = false, has_ym = false;

      if (cx + 1 < static_cast<int>(ground_size_x_)) {
        size_t nidx = groundIndex(static_cast<unsigned int>(cx + 1),
                                  static_cast<unsigned int>(cy));
        if (ground_map_[nidx].has_ground) {
          z_xp = ground_map_[nidx].ground_z;
          has_xp = true;
        }
      }
      if (cx > 0) {
        size_t nidx = groundIndex(static_cast<unsigned int>(cx - 1),
                                  static_cast<unsigned int>(cy));
        if (ground_map_[nidx].has_ground) {
          z_xm = ground_map_[nidx].ground_z;
          has_xm = true;
        }
      }
      if (cy + 1 < static_cast<int>(ground_size_y_)) {
        size_t nidx = groundIndex(static_cast<unsigned int>(cx),
                                  static_cast<unsigned int>(cy + 1));
        if (ground_map_[nidx].has_ground) {
          z_yp = ground_map_[nidx].ground_z;
          has_yp = true;
        }
      }
      if (cy > 0) {
        size_t nidx = groundIndex(static_cast<unsigned int>(cx),
                                  static_cast<unsigned int>(cy - 1));
        if (ground_map_[nidx].has_ground) {
          z_ym = ground_map_[nidx].ground_z;
          has_ym = true;
        }
      }

      if (has_xp && has_xm) {
        slope_x = (z_xp - z_xm) / (2.0f * res);
      } else if (has_xp) {
        slope_x = (z_xp - z_center) / res;
      } else if (has_xm) {
        slope_x = (z_center - z_xm) / res;
      }

      if (has_yp && has_ym) {
        slope_y = (z_yp - z_ym) / (2.0f * res);
      } else if (has_yp) {
        slope_y = (z_yp - z_center) / res;
      } else if (has_ym) {
        slope_y = (z_center - z_ym) / res;
      }

      ground_map_[idx].slope_x = slope_x;
      ground_map_[idx].slope_y = slope_y;
      ground_map_[idx].slope_magnitude = std::sqrt(slope_x * slope_x + slope_y * slope_y);

      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) continue;
          int nx = cx + dx;
          int ny = cy + dy;
          if (nx < 0 || nx >= static_cast<int>(ground_size_x_) ||
              ny < 0 || ny >= static_cast<int>(ground_size_y_))
          {
            continue;
          }
          size_t nidx = groundIndex(static_cast<unsigned int>(nx),
                                    static_cast<unsigned int>(ny));
          if (!ground_map_[nidx].has_ground) continue;
          float diff = std::abs(ground_map_[nidx].ground_z - z_center);
          if (diff > max_diff) max_diff = diff;
        }
      }
      ground_map_[idx].height_diff = max_diff;
    }
  }
}

unsigned char TraversabilityLayer::computeCost(const GroundCell & cell) const
{
  if (!cell.has_ground) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  if (cell.obstacle_ratio >= static_cast<float>(obstacle_ratio_threshold_)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  unsigned char obstacle_cost = 0;
  if (cell.obstacle_ratio > 0.0f) {
    float obs_norm = cell.obstacle_ratio / static_cast<float>(obstacle_ratio_threshold_);
    if (obs_norm > 1.0f) obs_norm = 1.0f;
    obstacle_cost = static_cast<unsigned char>(std::min(252.0f, obs_norm * 200.0f));
  }

  float height_above_ground = cell.height_diff;

  if (height_above_ground <= static_cast<float>(height_cost_start_)) {
    float slope_angle = std::atan(cell.slope_magnitude);
    if (slope_angle <= static_cast<float>(slope_cost_start_)) {
      return obstacle_cost;
    }
    float slope_range = static_cast<float>(max_slope_traversable_) - static_cast<float>(slope_cost_start_);
    if (slope_range > 1e-6f) {
      float slope_norm = (slope_angle - static_cast<float>(slope_cost_start_)) / slope_range;
      if (slope_norm > 1.0f) slope_norm = 1.0f;
      float cost_f = slope_norm * static_cast<float>(slope_cost_scale_) * 25.0f;
      unsigned char slope_only = static_cast<unsigned char>(std::min(252.0f, cost_f));
      return std::max(slope_only, obstacle_cost);
    }
    return obstacle_cost;
  }

  float h_range = static_cast<float>(step_height_threshold_) - static_cast<float>(height_cost_start_);
  float h_norm = height_above_ground - static_cast<float>(height_cost_start_);

  unsigned char height_cost = 0;
  if (h_range > 1e-6f) {
    float ratio = h_norm / h_range;
    if (ratio > 1.0f) ratio = 1.0f;
    float cost_f = 1.0f + ratio * 252.0f;
    height_cost = static_cast<unsigned char>(std::min(253.0f, cost_f));
  } else {
    height_cost = static_cast<unsigned char>(
      std::min(253.0f, (height_above_ground / static_cast<float>(step_height_threshold_)) * 253.0f));
  }

  if (height_above_ground > static_cast<float>(step_height_threshold_)) {
    float slope_angle = std::atan(cell.slope_magnitude);
    if (slope_angle > static_cast<float>(max_slope_traversable_)) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  }

  float slope_angle = std::atan(cell.slope_magnitude);
  if (slope_angle > static_cast<float>(slope_cost_start_)) {
    float slope_range = static_cast<float>(max_slope_traversable_) - static_cast<float>(slope_cost_start_);
    if (slope_range > 1e-6f) {
      float slope_norm = (slope_angle - static_cast<float>(slope_cost_start_)) / slope_range;
      if (slope_norm > 1.0f) slope_norm = 1.0f;
      float slope_cost_f = slope_norm * static_cast<float>(slope_cost_scale_) * 25.0f;
      unsigned char slope_cost = static_cast<unsigned char>(std::min(252.0f, slope_cost_f));
      return std::max({height_cost, slope_cost, obstacle_cost});
    }
  }

  return std::max(height_cost, obstacle_cost);
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

  if (new_gx != ground_size_x_ || new_gy != ground_size_y_) {
    ground_size_x_ = new_gx;
    ground_size_y_ = new_gy;
    ground_map_.resize(ground_size_x_ * ground_size_y_);
  }

  voxel_size_x_ = ground_size_x_;
  voxel_size_y_ = ground_size_y_;

  if (!voxel_grid_valid_) {
    ground_map_.assign(ground_size_x_ * ground_size_y_, GroundCell{});
    return;
  }

  decayVoxelGrid();

  extractGround(ox, oy);

  interpolateGround();

  computeGroundSlope();

  unsigned char * master_array = master_grid.getCharMap();
  int cells_with_cost = 0;
  int lethal_cells = 0;
  double inv_costmap_res = 1.0 / costmap_res;
  size_t master_size = static_cast<size_t>(costmap_sx) * static_cast<size_t>(costmap_sy);

  if (layer_costmap_sx_ != costmap_sx || layer_costmap_sy_ != costmap_sy) {
    layer_costmap_sx_ = costmap_sx;
    layer_costmap_sy_ = costmap_sy;
    layer_costmap_.resize(master_size);
  }
  layer_costmap_.assign(master_size, nav2_costmap_2d::NO_INFORMATION);

  int n_threads = omp_get_max_threads();
  std::vector<std::vector<unsigned char>> local_costmaps(n_threads);
  for (int t = 0; t < n_threads; t++) {
    local_costmaps[t].assign(master_size, nav2_costmap_2d::NO_INFORMATION);
  }

#pragma omp parallel for collapse(2) reduction(+:cells_with_cost) reduction(+:lethal_cells)
  for (unsigned int cy = 0; cy < ground_size_y_; cy++) {
    for (unsigned int cx = 0; cx < ground_size_x_; cx++) {
      size_t idx = groundIndex(cx, cy);
      const auto & cell = ground_map_[idx];

      if (!cell.has_ground) {
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

      int tid = omp_get_thread_num();
      auto & local_cm = local_costmaps[tid];

      for (int my = my_start; my <= my_end; my++) {
        for (int mx = mx_start; mx <= mx_end; mx++) {
          if (mx < 0 || my < 0) continue;

          unsigned int master_idx = master_grid.getIndex(
            static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
          if (cost > local_cm[master_idx]) {
            local_cm[master_idx] = cost;
          }
        }
      }
    }
  }

  for (int t = 0; t < n_threads; t++) {
    const auto & local_cm = local_costmaps[t];
    for (size_t i = 0; i < master_size; i++) {
      if (local_cm[i] != nav2_costmap_2d::NO_INFORMATION) {
        if (layer_costmap_[i] == nav2_costmap_2d::NO_INFORMATION || local_cm[i] > layer_costmap_[i]) {
          layer_costmap_[i] = local_cm[i];
        }
      }
    }
  }

  for (size_t i = 0; i < master_size; i++) {
    if (layer_costmap_[i] != nav2_costmap_2d::NO_INFORMATION) {
      master_array[i] = layer_costmap_[i];
    }
  }

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("traversability_layer"), *clock, 2000,
    "[TraversabilityLayer] updateCosts: %d cells with cost, %d lethal",
    cells_with_cost, lethal_cells);

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
      "cells_with_cost=%d, lethal=%d, voxel_grid=%.1fMB, frame=%u",
      perf_frame_count_, elapsed, avg_ms, frame_ms,
      cells_with_cost, lethal_cells,
      static_cast<double>(voxel_grid_.size() * sizeof(VoxelData)) / (1024.0 * 1024.0),
      static_cast<unsigned int>(frame_counter_));
    perf_frame_count_ = 0;
    perf_total_time_ = 0.0;
    last_perf_log_ = frame_end;
  }

  if (publish_slope_map_ && slope_pub_) {
    auto node = node_.lock();
    if (node) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      slope_cloud->header.frame_id = layered_costmap_->getGlobalFrameID();

      for (unsigned int cy = 0; cy < ground_size_y_; cy++) {
        for (unsigned int cx = 0; cx < ground_size_x_; cx++) {
          size_t idx = groundIndex(cx, cy);
          const auto & cell = ground_map_[idx];

          if (!cell.has_ground) {
            continue;
          }

          pcl::PointXYZI pt;
          pt.x = static_cast<float>(cx * cell_resolution_ + ox);
          pt.y = static_cast<float>(cy * cell_resolution_ + oy);
          pt.z = cell.ground_z;
          unsigned char cost = computeCost(cell);
          pt.intensity = static_cast<float>(cost) / 254.0f;
          slope_cloud->push_back(pt);
        }
      }

      sensor_msgs::msg::PointCloud2 slope_msg;
      pcl::toROSMsg(*slope_cloud, slope_msg);
      slope_msg.header.stamp = node->now();
      slope_pub_->publish(slope_msg);
    }
  }

  cloud_updated_ = false;
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
}

void TraversabilityLayer::resetMaps()
{
  ground_map_.assign(ground_size_x_ * ground_size_y_, GroundCell{});
  voxel_grid_.clear();
  voxel_grid_valid_ = false;
  frame_counter_ = 0;
  cloud_updated_ = false;
}

void TraversabilityLayer::resetLayerCostmap()
{
  layer_costmap_.assign(layer_costmap_sx_ * layer_costmap_sy_, nav2_costmap_2d::NO_INFORMATION);
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
