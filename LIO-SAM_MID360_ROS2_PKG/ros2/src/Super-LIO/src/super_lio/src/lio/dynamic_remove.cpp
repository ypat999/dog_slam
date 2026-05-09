/**
 * @file dynamic_remove.cpp
 * @brief Dynamic point removal from point cloud frames
 * 
 * This module provides two methods for dynamic point removal:
 * 
 * Method 1 (TEMPORAL): Builds occupancy grids for each frame and filters out points
 * that are not occupied in both previous and next frames.
 * 
 * Method 2 (RAYCAST): Merges all frames first, then for each frame, performs raycast
 * from sensor position to each point. Marks voxels that are penetrated by rays and
 * removes those points from the merged cloud.
 */

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <fstream>
#include <sstream>
#include <mutex>
#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>

namespace fs = std::filesystem;

namespace DynamicRemove {

enum class RemovalMethod {
    TEMPORAL = 0,
    RAYCAST = 1
};

struct Config {
    float grid_size = 0.2f;
    int min_neighbor_grids = 2;
    int frame_window = 1;
    std::string input_dir = "./PCD";
    std::string output_file = "./filtered_map.pcd";
    bool enable_isolated_removal = true;
    bool verbose = true;
    RemovalMethod method = RemovalMethod::TEMPORAL;
    int raycast_min_hits = 2;
    std::string scans_prefix = "";
};

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
    }
};

struct OdomData {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};

inline VoxelKey pointToVoxelKey(const PointType& point, float grid_size) {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.x / grid_size));
    key.y = static_cast<int>(std::floor(point.y / grid_size));
    key.z = static_cast<int>(std::floor(point.z / grid_size));
    return key;
}

inline VoxelKey positionToVoxelKey(const Eigen::Vector3f& pos, float grid_size) {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(pos.x() / grid_size));
    key.y = static_cast<int>(std::floor(pos.y() / grid_size));
    key.z = static_cast<int>(std::floor(pos.z() / grid_size));
    return key;
}

class OccupancyGrid {
public:
    explicit OccupancyGrid(float grid_size) : grid_size_(grid_size) {}

    void insertCloud(const CloudPtr& cloud) {
        for (const auto& point : cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            VoxelKey key = pointToVoxelKey(point, grid_size_);
            occupied_voxels_.insert(key);
        }
    }

    bool isOccupied(const VoxelKey& key) const {
        return occupied_voxels_.find(key) != occupied_voxels_.end();
    }

    const std::unordered_set<VoxelKey, VoxelKeyHash>& getOccupiedVoxels() const {
        return occupied_voxels_;
    }

    void clear() {
        occupied_voxels_.clear();
    }

    size_t size() const {
        return occupied_voxels_.size();
    }

private:
    float grid_size_;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
};

struct FrameData {
    CloudPtr cloud;
    OdomData odom;
    int index;
};

std::vector<FrameData> loadPointCloudFramesWithOdom(const std::string& input_dir, bool verbose) {
    std::vector<FrameData> frames;
    std::vector<std::pair<std::string, std::string>> pcd_txt_files;

    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.path().extension() == ".pcd") {
            std::string pcd_file = entry.path().string();
            std::string txt_file = pcd_file.substr(0, pcd_file.size() - 4) + ".txt";
            
            if (fs::exists(txt_file)) {
                pcd_txt_files.push_back({pcd_file, txt_file});
            } else {
                if (verbose) {
                    LOG(WARNING) << "No odom file for: " << pcd_file;
                }
            }
        }
    }

    std::sort(pcd_txt_files.begin(), pcd_txt_files.end(), 
        [](const auto& a, const auto& b) { return a.first < b.first; });

    if (verbose) {
        LOG(INFO) << "Found " << pcd_txt_files.size() << " PCD files with odom in " << input_dir;
    }

    int index = 0;
    for (const auto& [pcd_file, txt_file] : pcd_txt_files) {
        CloudPtr cloud(new PointCloudType());
        if (pcl::io::loadPCDFile<PointType>(pcd_file, *cloud) == 0) {
            CloudPtr valid_cloud(new PointCloudType());
            for (const auto& pt : cloud->points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    valid_cloud->points.push_back(pt);
                }
            }
            valid_cloud->width = valid_cloud->points.size();
            valid_cloud->height = 1;
            valid_cloud->is_dense = true;

            OdomData odom;
            std::ifstream odom_stream(txt_file);
            if (odom_stream.is_open()) {
                float qx, qy, qz, qw;
                odom_stream >> odom.timestamp 
                           >> odom.position.x() >> odom.position.y() >> odom.position.z()
                           >> qx >> qy >> qz >> qw;
                odom.orientation = Eigen::Quaternionf(qw, qx, qy, qz);
                odom.orientation.normalize();
                odom_stream.close();
            } else {
                LOG(WARNING) << "Failed to read odom: " << txt_file;
                continue;
            }

            FrameData frame;
            frame.cloud = valid_cloud;
            frame.odom = odom;
            frame.index = index++;
            frames.push_back(frame);

            if (verbose) {
                LOG(INFO) << "Loaded: " << pcd_file << " (" << valid_cloud->size() 
                          << " points, odom: " << odom.position.transpose() << ")";
            }
        } else {
            LOG(WARNING) << "Failed to load: " << pcd_file;
        }
    }

    return frames;
}

std::vector<CloudPtr> loadPointCloudFrames(const std::string& input_dir, bool verbose, const std::string& scans_prefix = "") {
    std::vector<CloudPtr> frames;
    std::vector<std::string> pcd_files;

    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.path().extension() == ".pcd") {
            std::string filename = entry.path().filename().string();
            if (scans_prefix.empty() || filename.find(scans_prefix) == 0) {
                pcd_files.push_back(entry.path().string());
            }
        }
    }

    std::sort(pcd_files.begin(), pcd_files.end());

    if (verbose) {
        LOG(INFO) << "Found " << pcd_files.size() << " PCD files in " << input_dir 
                  << (scans_prefix.empty() ? "" : " with prefix '" + scans_prefix + "'");
    }

    for (const auto& file : pcd_files) {
        CloudPtr cloud(new PointCloudType());
        if (pcl::io::loadPCDFile<PointType>(file, *cloud) == 0) {
            CloudPtr valid_cloud(new PointCloudType());
            for (const auto& pt : cloud->points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    valid_cloud->points.push_back(pt);
                }
            }
            valid_cloud->width = valid_cloud->points.size();
            valid_cloud->height = 1;
            valid_cloud->is_dense = true;
            frames.push_back(valid_cloud);
            if (verbose) {
                LOG(INFO) << "Loaded: " << file << " (" << valid_cloud->size() << " points)";
            }
        } else {
            LOG(WARNING) << "Failed to load: " << file;
        }
    }

    return frames;
}

CloudPtr filterDynamicPointsTemporal(
    const std::vector<CloudPtr>& frames,
    const Config& config)
{
    if (frames.empty()) {
        LOG(WARNING) << "No frames to process";
        return CloudPtr(new PointCloudType());
    }

    size_t n_frames = frames.size();
    std::vector<OccupancyGrid> grids(n_frames, OccupancyGrid(config.grid_size));

#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < n_frames; ++i) {
        grids[i].insertCloud(frames[i]);
        if (config.verbose && i % 10 == 0) {
#pragma omp critical
            LOG(INFO) << "Frame " << i << ": " << grids[i].size() << " occupied voxels";
        }
    }

    std::vector<PointType, Eigen::aligned_allocator<PointType>> filtered_points;
    std::vector<int> frame_removed_counts(n_frames, 0);
    int total_points = 0;
    int removed_points = 0;

#pragma omp parallel reduction(+:total_points, removed_points)
    {
        std::vector<PointType, Eigen::aligned_allocator<PointType>> local_filtered_points;

#pragma omp for schedule(dynamic)
        for (size_t frame_idx = 0; frame_idx < n_frames; ++frame_idx) {
            const CloudPtr& frame = frames[frame_idx];
            int frame_removed = 0;

            for (const auto& point : frame->points) {
                total_points++;
                VoxelKey key = pointToVoxelKey(point, config.grid_size);

                bool is_dynamic = true;
                int window = config.frame_window;

                for (int offset = -window; offset <= window; ++offset) {
                    if (offset == 0) continue;
                    
                    int neighbor_idx = static_cast<int>(frame_idx) + offset;
                    if (neighbor_idx < 0 || neighbor_idx >= static_cast<int>(n_frames)) {
                        continue;
                    }

                    if (grids[neighbor_idx].isOccupied(key)) {
                        is_dynamic = false;
                        break;
                    }
                }

                if (is_dynamic) {
                    removed_points++;
                    frame_removed++;
                } else {
                    local_filtered_points.push_back(point);
                }
            }

            frame_removed_counts[frame_idx] = frame_removed;
        }

#pragma omp critical
        filtered_points.insert(filtered_points.end(), local_filtered_points.begin(), local_filtered_points.end());
    }

    for (size_t frame_idx = 0; frame_idx < n_frames; ++frame_idx) {
        if (config.verbose && frame_removed_counts[frame_idx] > 0) {
            LOG(INFO) << "Frame " << frame_idx << ": removed " << frame_removed_counts[frame_idx] << " dynamic points";
        }
    }

    CloudPtr filtered_cloud(new PointCloudType());
    filtered_cloud->points = std::move(filtered_points);
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    if (config.verbose) {
        LOG(INFO) << "Temporal dynamic removal: " << removed_points << " / " << total_points 
                  << " points removed (" << (100.0 * removed_points / total_points) << "%)";
    }

    return filtered_cloud;
}

std::vector<VoxelKey> raycastVoxels(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    float grid_size)
{
    std::vector<VoxelKey> voxels;
    
    VoxelKey start_key = positionToVoxelKey(start, grid_size);
    VoxelKey end_key = positionToVoxelKey(end, grid_size);
    
    int dx = end_key.x - start_key.x;
    int dy = end_key.y - start_key.y;
    int dz = end_key.z - start_key.z;
    
    int step_x = (dx >= 0) ? 1 : -1;
    int step_y = (dy >= 0) ? 1 : -1;
    int step_z = (dz >= 0) ? 1 : -1;
    
    int abs_dx = std::abs(dx);
    int abs_dy = std::abs(dy);
    int abs_dz = std::abs(dz);
    
    int max_dist = abs_dx + abs_dy + abs_dz;
    if (max_dist == 0) return voxels;
    
    double t_delta_x = (dx != 0) ? (double)grid_size / std::abs(end.x() - start.x()) : 1e10;
    double t_delta_y = (dy != 0) ? (double)grid_size / std::abs(end.y() - start.y()) : 1e10;
    double t_delta_z = (dz != 0) ? (double)grid_size / std::abs(end.z() - start.z()) : 1e10;
    
    double t_max_x = (dx != 0) ? 
        ((start_key.x + step_x) * grid_size - start.x()) / (end.x() - start.x()) : 1e10;
    double t_max_y = (dy != 0) ? 
        ((start_key.y + step_y) * grid_size - start.y()) / (end.y() - start.y()) : 1e10;
    double t_max_z = (dz != 0) ? 
        ((start_key.z + step_z) * grid_size - start.z()) / (end.z() - start.z()) : 1e10;
    
    int x = start_key.x;
    int y = start_key.y;
    int z = start_key.z;
    
    for (int i = 0; i < max_dist * 2 + 1; ++i) {
        if (x != end_key.x || y != end_key.y || z != end_key.z) {
            voxels.push_back({x, y, z});
        }
        
        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                x += step_x;
                t_max_x += t_delta_x;
            } else {
                z += step_z;
                t_max_z += t_delta_z;
            }
        } else {
            if (t_max_y < t_max_z) {
                y += step_y;
                t_max_y += t_delta_y;
            } else {
                z += step_z;
                t_max_z += t_delta_z;
            }
        }
        
        if (t_max_x > 1.0 && t_max_y > 1.0 && t_max_z > 1.0) break;
    }
    
    return voxels;
}

CloudPtr filterDynamicPointsRaycast(
    const std::vector<FrameData>& frames,
    const Config& config)
{
    if (frames.empty()) {
        LOG(WARNING) << "No frames to process";
        return CloudPtr(new PointCloudType());
    }

    LOG(INFO) << "=== Raycast Method: Building global occupancy grid ===";
    
    CloudPtr merged_cloud(new PointCloudType());
    for (const auto& frame : frames) {
        *merged_cloud += *frame.cloud;
    }
    
    OccupancyGrid global_grid(config.grid_size);
    global_grid.insertCloud(merged_cloud);
    
    if (config.verbose) {
        LOG(INFO) << "Merged cloud: " << merged_cloud->size() << " points";
        LOG(INFO) << "Global grid: " << global_grid.size() << " occupied voxels";
    }

    LOG(INFO) << "=== Raycast Method: Counting observations and penetrations ===";
    
    std::unordered_map<VoxelKey, int, VoxelKeyHash> observation_count;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> penetration_count;
    std::mutex obs_mutex, pen_mutex;
    
    const auto& occupied_voxels = global_grid.getOccupiedVoxels();
    
    for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
        const auto& frame = frames[frame_idx];
        const Eigen::Vector3f& sensor_pos = frame.odom.position;
        VoxelKey sensor_key = positionToVoxelKey(sensor_pos, config.grid_size);
        
        int frame_observations = 0;
        int frame_penetrations = 0;
        
        std::unordered_map<VoxelKey, int, VoxelKeyHash> local_obs;
        std::unordered_map<VoxelKey, int, VoxelKeyHash> local_pen;
        
        for (const auto& point : frame.cloud->points) {
            Eigen::Vector3f point_pos(point.x, point.y, point.z);
            VoxelKey point_key = pointToVoxelKey(point, config.grid_size);
            
            local_obs[point_key]++;
            
            if (point_key.x == sensor_key.x && point_key.y == sensor_key.y && point_key.z == sensor_key.z) {
                continue;
            }
            
            std::vector<VoxelKey> ray_voxels = raycastVoxels(sensor_pos, point_pos, config.grid_size);
            
            for (const auto& voxel_key : ray_voxels) {
                if (occupied_voxels.find(voxel_key) != occupied_voxels.end()) {
                    local_pen[voxel_key]++;
                    frame_penetrations++;
                }
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(obs_mutex);
            for (const auto& [key, count] : local_obs) {
                observation_count[key] += count;
            }
        }
        {
            std::lock_guard<std::mutex> lock(pen_mutex);
            for (const auto& [key, count] : local_pen) {
                penetration_count[key] += count;
            }
        }
        
        frame_observations = local_obs.size();
        

        LOG(INFO) << "Frame " << frame_idx << ": " << frame_observations 
                << " observed, " << frame_penetrations << " penetrations";

    }

    LOG(INFO) << "=== Raycast Method: Filtering dynamic voxels ===";
    
    std::unordered_set<VoxelKey, VoxelKeyHash> dynamic_voxels;
    int total_checked = 0;
    
    for (const auto& voxel_key : occupied_voxels) {
        int obs = observation_count[voxel_key];
        int pen = penetration_count[voxel_key];
        
        if (obs > 0) {
            total_checked++;
            float ratio = static_cast<float>(pen) / static_cast<float>(obs);
            
            if (pen >= config.raycast_min_hits && ratio > 0.5f) {
                dynamic_voxels.insert(voxel_key);
            }
        }
    }
    
    if (config.verbose) {
        LOG(INFO) << "Dynamic voxels (pen>=" << config.raycast_min_hits 
                  << ", ratio>0.5): " << dynamic_voxels.size();
    }

    CloudPtr filtered_cloud(new PointCloudType());
    int total_points = merged_cloud->size();
    int removed_points = 0;
    
    for (const auto& point : merged_cloud->points) {
        VoxelKey key = pointToVoxelKey(point, config.grid_size);
        
        if (dynamic_voxels.find(key) != dynamic_voxels.end()) {
            removed_points++;
        } else {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    if (config.verbose) {
        LOG(INFO) << "Raycast dynamic removal: " << removed_points << " / " << total_points 
                  << " points removed (" << (100.0 * removed_points / total_points) << "%)";
    }
    
    return filtered_cloud;
}

CloudPtr removeIsolatedPoints(
    const CloudPtr& cloud,
    const Config& config)
{
    if (!config.enable_isolated_removal || cloud->empty()) {
        return cloud;
    }

    OccupancyGrid grid(config.grid_size);
    grid.insertCloud(cloud);

    CloudPtr filtered_cloud(new PointCloudType());
    int total_points = cloud->size();
    int isolated_points = 0;

    const std::vector<std::tuple<int, int, int>> neighbor_offsets = {
        {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1},
        {-1, 0, -1},  {-1, 0, 0},  {-1, 0, 1},
        {-1, 1, -1},  {-1, 1, 0},  {-1, 1, 1},
        {0, -1, -1},  {0, -1, 0},  {0, -1, 1},
        {0, 0, -1},   {0, 0, 1},
        {0, 1, -1},   {0, 1, 0},   {0, 1, 1},
        {1, -1, -1},  {1, -1, 0},  {1, -1, 1},
        {1, 0, -1},   {1, 0, 0},   {1, 0, 1},
        {1, 1, -1},   {1, 1, 0},   {1, 1, 1}
    };

    for (const auto& point : cloud->points) {
        VoxelKey key = pointToVoxelKey(point, config.grid_size);
        
        int neighbor_count = 0;
        for (const auto& [dx, dy, dz] : neighbor_offsets) {
            VoxelKey neighbor_key{key.x + dx, key.y + dy, key.z + dz};
            if (grid.isOccupied(neighbor_key)) {
                neighbor_count++;
            }
        }

        if (neighbor_count < config.min_neighbor_grids) {
            isolated_points++;
        } else {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    if (config.verbose) {
        LOG(INFO) << "Isolated removal: " << isolated_points << " / " << total_points 
                  << " points removed (" << (100.0 * isolated_points / total_points) << "%)";
    }

    return filtered_cloud;
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "Options:\n"
              << "  --input_dir <path>       Input directory containing PCD files (default: ./PCD)\n"
              << "  --output_file <path>     Output PCD file path (default: ./filtered_map.pcd)\n"
              << "  --grid_size <float>      Voxel grid size in meters (default: 0.2)\n"
              << "  --min_neighbors <int>    Minimum neighbor grids to keep a point (default: 2)\n"
              << "  --frame_window <int>     Frame window size for temporal method (default: 1)\n"
              << "  --method <0|1>           Removal method: 0=Temporal, 1=Raycast (default: 0)\n"
              << "  --raycast_min_hits <int> Min hits for raycast method (default: 2)\n"
              << "  --scans_prefix <str>     Only process PCD files with this prefix (default: empty)\n"
              << "  --disable_isolated       Disable isolated point removal\n"
              << "  --quiet                  Reduce output verbosity\n"
              << "  --help                   Show this help message\n";
}

Config parseArgs(int argc, char** argv) {
    Config config;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help") {
            printUsage(argv[0]);
            exit(0);
        } else if (arg == "--input_dir" && i + 1 < argc) {
            config.input_dir = argv[++i];
        } else if (arg == "--output_file" && i + 1 < argc) {
            config.output_file = argv[++i];
        } else if (arg == "--grid_size" && i + 1 < argc) {
            config.grid_size = std::stof(argv[++i]);
        } else if (arg == "--min_neighbors" && i + 1 < argc) {
            config.min_neighbor_grids = std::stoi(argv[++i]);
        } else if (arg == "--frame_window" && i + 1 < argc) {
            config.frame_window = std::stoi(argv[++i]);
        } else if (arg == "--method" && i + 1 < argc) {
            int method = std::stoi(argv[++i]);
            config.method = (method == 1) ? RemovalMethod::RAYCAST : RemovalMethod::TEMPORAL;
        } else if (arg == "--raycast_min_hits" && i + 1 < argc) {
            config.raycast_min_hits = std::stoi(argv[++i]);
        } else if (arg == "--scans_prefix" && i + 1 < argc) {
            config.scans_prefix = argv[++i];
        } else if (arg == "--disable_isolated") {
            config.enable_isolated_removal = false;
        } else if (arg == "--quiet") {
            config.verbose = false;
        } else {
            LOG(WARNING) << "Unknown argument: " << arg;
        }
    }
    
    return config;
}

void runDynamicRemoval(const Config& config) {
    LOG(INFO) << "=== Dynamic Point Removal Configuration ===";
    LOG(INFO) << "Input directory: " << config.input_dir;
    LOG(INFO) << "Output file: " << config.output_file;
    LOG(INFO) << "Grid size: " << config.grid_size << " m";
    LOG(INFO) << "Min neighbor grids: " << config.min_neighbor_grids;
    LOG(INFO) << "Method: " << (config.method == RemovalMethod::TEMPORAL ? "Temporal" : "Raycast");
    
    if (config.method == RemovalMethod::TEMPORAL) {
        LOG(INFO) << "Frame window: " << config.frame_window;
    } else {
        LOG(INFO) << "Raycast min hits: " << config.raycast_min_hits;
    }
    LOG(INFO) << "Isolated removal: " << (config.enable_isolated_removal ? "enabled" : "disabled");

    CloudPtr filtered_cloud(new PointCloudType());

    if (config.method == RemovalMethod::RAYCAST) {
        LOG(INFO) << "\n=== Loading Point Cloud Frames with Odom ===";
        std::vector<FrameData> frames = loadPointCloudFramesWithOdom(config.input_dir, config.verbose);
        
        if (frames.empty()) {
            LOG(ERROR) << "No valid point cloud frames with odom loaded. Exiting.";
            return;
        }

        LOG(INFO) << "\n=== Filtering Dynamic Points (Raycast Method) ===";
        filtered_cloud = filterDynamicPointsRaycast(frames, config);
    } else {
        LOG(INFO) << "\n=== Loading Point Cloud Frames ===";
        std::vector<CloudPtr> frames = loadPointCloudFrames(config.input_dir, config.verbose, config.scans_prefix);
        
        if (frames.empty()) {
            LOG(ERROR) << "No valid point cloud frames loaded. Exiting.";
            return;
        }

        LOG(INFO) << "\n=== Filtering Dynamic Points (Temporal Method) ===";
        filtered_cloud = filterDynamicPointsTemporal(frames, config);
    }

    LOG(INFO) << "\n=== Removing Isolated Points ===";
    CloudPtr final_cloud = removeIsolatedPoints(filtered_cloud, config);

    LOG(INFO) << "\n=== Saving Result ===";
    if (final_cloud->empty()) {
        LOG(WARNING) << "No points remaining after filtering. Output file not created.";
        return;
    }

    if (pcl::io::savePCDFileBinary(config.output_file, *final_cloud) == 0) {
        LOG(INFO) << "Successfully saved filtered point cloud to: " << config.output_file;
        LOG(INFO) << "Final point count: " << final_cloud->size();
    } else {
        LOG(ERROR) << "Failed to save point cloud to: " << config.output_file;
    }
}

}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    DynamicRemove::Config config = DynamicRemove::parseArgs(argc, argv);
    DynamicRemove::runDynamicRemoval(config);

    google::ShutdownGoogleLogging();
    return 0;
}
