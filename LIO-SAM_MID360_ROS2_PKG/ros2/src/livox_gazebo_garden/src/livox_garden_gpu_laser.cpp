#include "livox_gazebo_garden/livox_garden_gpu_laser.h"

#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/Sensor.hh>
#include <sdf/Lidar.hh>

#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/plugin/Register.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>

namespace livox_gazebo_garden
{

LivoxGpuLaserSystem::LivoxGpuLaserSystem()
{
}

LivoxGpuLaserSystem::~LivoxGpuLaserSystem() = default;

void LivoxGpuLaserSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  this->robotNamespace_ = _sdf->Get<std::string>("robot_namespace", "").first;
  this->csvFileName_ = _sdf->Get<std::string>("csv_file_name", "").first;

  if (this->csvFileName_.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("livox_gpu_laser"),
        "CSV file name parameter (csv_file_name) is required but not provided!");
    return;
  }

  std::string resolvedPath = ResolvePackageURI(this->csvFileName_);
  LoadCsvPattern(resolvedPath);

  this->rosNode_ = rclcpp::Node::make_shared("livox_garden_gpu_laser");
  this->gzNode_ = std::make_unique<gz::transport::Node>();

  RCLCPP_INFO(rclcpp::get_logger("livox_gpu_laser"),
      "Livox Gazebo Garden GPU Laser System configured");
}

std::string LivoxGpuLaserSystem::ResolvePackageURI(const std::string &uri)
{
  if (uri.find("package://") == 0)
  {
    std::string path = uri.substr(10);
    size_t slashPos = path.find('/');
    if (slashPos != std::string::npos)
    {
      std::string packageName = path.substr(0, slashPos);
      std::string relativePath = path.substr(slashPos + 1);
      try
      {
        std::string packagePath =
            ament_index_cpp::get_package_share_directory(packageName);
        return packagePath + "/" + relativePath;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("livox_gpu_laser"),
            "Failed to resolve package:// URI '%s': %s",
            uri.c_str(), e.what());
        return uri;
      }
    }
  }
  return uri;
}

void LivoxGpuLaserSystem::LoadCsvPattern(const std::string &csv_path)
{
  RCLCPP_INFO(rclcpp::get_logger("livox_gpu_laser"),
      "Loading CSV pattern from: %s", csv_path.c_str());

  std::ifstream file(csv_path);
  if (!file.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("livox_gpu_laser"),
        "Could not open CSV file: %s", csv_path.c_str());
    return;
  }

  SensorData defaultSensor;
  defaultSensor.samplesPerUpdate = 1000;
  defaultSensor.downsample = 1;

  std::string line;
  std::getline(file, line);

  int lineCount = 0;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string value;
    ScanPatternPoint point;

    std::getline(ss, value, ',');
    point.time = std::stod(value);
    std::getline(ss, value, ',');
    point.azimuth = std::stod(value) * M_PI / 180.0;
    std::getline(ss, value, ',');
    point.zenith = (90 - std::stod(value)) * M_PI / 180.0;

    defaultSensor.scanPattern.push_back(point);
    lineCount++;
  }
  file.close();

  defaultSensor.patternIndex = 0;
  this->sensorDataList_.push_back(std::move(defaultSensor));

  RCLCPP_INFO(rclcpp::get_logger("livox_gpu_laser"),
      "Loaded %d scan pattern points from CSV", lineCount);
}

void LivoxGpuLaserSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  _ecm.Each<gz::sim::components::Sensor,
             gz::sim::components::ParentEntity,
             gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Sensor *,
          const gz::sim::components::ParentEntity *,
          const gz::sim::components::Name *)->bool
      {
        bool found = false;
        for (auto &sd : this->sensorDataList_)
        {
          if (sd.sensorEntity == _entity)
          {
            found = true;
            break;
          }
        }

        if (!found && !this->sensorDataList_.empty())
        {
          gz::sim::Sensor simSensor(_entity);

          SensorData newSd;
          newSd.sensorEntity = _entity;
          newSd.scanPattern = this->sensorDataList_[0].scanPattern;
          newSd.samplesPerUpdate = this->sensorDataList_[0].samplesPerUpdate;
          newSd.downsample = this->sensorDataList_[0].downsample;
          newSd.patternIndex = 0;

          auto topicOpt = simSensor.Topic(_ecm);
          newSd.topicName = topicOpt.value_or("");

          auto nameOpt = simSensor.Name(_ecm);
          newSd.frameName = nameOpt.value_or("");

          auto sdfComp = _ecm.Component<gz::sim::components::GpuLidar>(_entity);
          if (sdfComp)
          {
            const sdf::Sensor &lidarSdf = sdfComp->Data();
            const sdf::Lidar *lidarElem = lidarSdf.LidarSensor();
            if (lidarElem)
            {
              newSd.samplesPerUpdate =
                  static_cast<int>(lidarElem->HorizontalScanSamples());
              newSd.downsample = 1;
            }
          }

          newSd.pub = this->rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
              newSd.topicName + "/ros2", 10);

          RCLCPP_INFO(rclcpp::get_logger("livox_gpu_laser"),
              "Registered GPU Lidar sensor [%s] -> topic [%s], frame [%s]",
              newSd.frameName.c_str(),
              newSd.topicName.c_str(),
              newSd.frameName.c_str());

          this->sensorDataList_.push_back(std::move(newSd));
        }
        return true;
      });
}

void LivoxGpuLaserSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  rclcpp::Time simTime(_info.simTime.count());

  for (auto &sd : this->sensorDataList_)
  {
    if (!sd.pub || sd.scanPattern.empty())
      continue;

    auto sdfComp = _ecm.Component<gz::sim::components::GpuLidar>(sd.sensorEntity);
    if (!sdfComp)
      continue;

    sensor_msgs::msg::PointCloud2 cloudMsg;
    cloudMsg.header.stamp = simTime;
    cloudMsg.header.frame_id = sd.frameName;
    cloudMsg.height = 1;
    cloudMsg.is_dense = true;
    cloudMsg.is_bigendian = false;
    cloudMsg.point_step = 16;

    struct Point3D { float x, y, z, intensity; };
    std::vector<Point3D> validPoints;
    validPoints.reserve(sd.samplesPerUpdate / sd.downsample);

    const sdf::Sensor &lidarSdf = sdfComp->Data();
    const sdf::Lidar *lidarElem = lidarSdf.LidarSensor();

    double minHorzAngle = lidarElem ? lidarElem->HorizontalScanResolution() : 0.01;
    double minVertAngle = lidarElem ? lidarElem->VerticalScanMinAngle().Radian() : -M_PI_4;
    double maxVertAngle = lidarElem ? lidarElem->VerticalScanMaxAngle().Radian() : M_PI_4;
    double vertAngleRes = lidarElem ? lidarElem->VerticalScanResolution() : 0.01;
    int rayCount = lidarElem ? static_cast<int>(lidarElem->HorizontalScanSamples()) : 1000;
    int vertRayCount = lidarElem ? static_cast<int>(lidarElem->VerticalScanSamples()) : 16;

    size_t patternSize = sd.scanPattern.size();

    for (int i = 0; i < sd.samplesPerUpdate; ++i)
    {
      size_t currentIndex = (sd.patternIndex + i) % patternSize;
      const auto &targetPoint = sd.scanPattern[currentIndex];

      if (targetPoint.zenith < minVertAngle ||
          targetPoint.zenith > maxVertAngle)
        continue;

      int u = static_cast<int>(std::round(
          targetPoint.azimuth / minHorzAngle));
      int v = static_cast<int>(std::round(
          (targetPoint.zenith - minVertAngle) / vertAngleRes));

      if (u < 0 || u >= rayCount || v < 0 || v >= vertRayCount)
        continue;

      validPoints.push_back(Point3D{
          static_cast<float>(targetPoint.azimuth),
          static_cast<float>(targetPoint.zenith),
          static_cast<float>(sd.maxDist),
          100.0f});
    }

    cloudMsg.width = validPoints.size();

    sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(validPoints.size());

    if (!validPoints.empty())
    {
      sensor_msgs::PointCloud2Iterator<float> iterX(cloudMsg, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(cloudMsg, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(cloudMsg, "z");
      sensor_msgs::PointCloud2Iterator<float> iterI(cloudMsg, "intensity");

      for (const auto &pt : validPoints)
      {
        *iterX = pt.x;
        *iterY = pt.y;
        *iterZ = pt.z;
        *iterI = pt.intensity;
        ++iterX; ++iterY; ++iterZ; ++iterI;
      }
    }

    sd.pub->publish(cloudMsg);
    sd.patternIndex = (sd.patternIndex + sd.samplesPerUpdate) % patternSize;
  }

  rclcpp::spin_some(this->rosNode_);
}

}

// GZ_ADD_PLUGIN(livox_gazebo_garden::LivoxGpuLaserSystem,
//     gz::sim::System,
//     livox_gazebo_garden::LivoxGpuLaserSystem::ISystemConfigure,
//     livox_gazebo_garden::LivoxGpuLaserSystem::ISystemPreUpdate,
//     livox_gazebo_garden::LivoxGpuLaserSystem::ISystemPostUpdate)

// GZ_ADD_PLUGIN_ALIAS(livox_gazebo_garden::LivoxGpuLaserSystem,
//     "livox_gazebo_garden::LivoxGpuLaserSystem")
