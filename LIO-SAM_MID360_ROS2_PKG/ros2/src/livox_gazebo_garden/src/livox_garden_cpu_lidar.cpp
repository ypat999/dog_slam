#include "livox_gazebo_garden/livox_garden_cpu_lidar.h"

#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/Sensor.hh>
#include <sdf/Lidar.hh>

#include <gz/plugin/Register.hh>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>

namespace livox_gazebo_garden
{

LivoxCpuLidarSystem::LivoxCpuLidarSystem()
{
}

LivoxCpuLidarSystem::~LivoxCpuLidarSystem() = default;

void LivoxCpuLidarSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  this->rosNode_ = rclcpp::Node::make_shared("livox_garden_cpu_lidar");
  this->gzNode_ = std::make_unique<gz::transport::Node>();

  RCLCPP_INFO(rclcpp::get_logger("livox_cpu_lidar"),
      "Livox Gazebo Garden CPU Lidar System configured");
}

std::string LivoxCpuLidarSystem::ResolvePackageURI(const std::string &uri)
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
        RCLCPP_ERROR(rclcpp::get_logger("livox_cpu_lidar"),
            "Failed to resolve package:// URI '%s': %s",
            uri.c_str(), e.what());
        return uri;
      }
    }
  }
  return uri;
}

bool LivoxCpuLidarSystem::ReadCsvFile(
    const std::string &file_name,
    std::vector<std::vector<double>> &datas)
{
  std::ifstream in_file(file_name);
  if (!in_file.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("livox_cpu_lidar"),
        "Could not open CSV file: %s", file_name.c_str());
    return false;
  }

  std::string line;
  while (std::getline(in_file, line))
  {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;
    while (std::getline(ss, value, ','))
    {
      row.push_back(std::stod(value));
    }
    datas.push_back(row);
  }

  in_file.close();
  return true;
}

void LivoxCpuLidarSystem::ConvertDataToRotateInfo(
    const std::vector<std::vector<double>> &datas,
    std::vector<AviaRotateInfo> &avia_infos)
{
  avia_infos.clear();
  for (size_t i = 1; i < datas.size(); ++i)
  {
    AviaRotateInfo info;
    info.time = datas[i][0];
    info.azimuth = datas[i][1] * M_PI / 180.0;
    info.zenith = (90 - datas[i][2]) * M_PI / 180.0;
    avia_infos.push_back(info);
  }
}

void LivoxCpuLidarSystem::PreUpdate(
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

        if (!found)
        {
          gz::sim::Sensor simSensor(_entity);

          SensorData newSd;
          newSd.sensorEntity = _entity;

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
              newSd.samplesStep =
                  static_cast<int>(lidarElem->HorizontalScanSamples());
              newSd.downSample = 1;
              newSd.minDist = lidarElem->RangeMin();
              newSd.maxDist = lidarElem->RangeMax();
            }
          }

          newSd.cloud2Pub =
              this->rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
                  newSd.topicName + "/cloud2", 10);
          newSd.customPub =
              this->rosNode_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
                  newSd.topicName, 10);

          RCLCPP_INFO(rclcpp::get_logger("livox_cpu_lidar"),
              "Registered CPU Lidar sensor [%s] -> topic [%s], frame [%s]",
              newSd.frameName.c_str(),
              newSd.topicName.c_str(),
              newSd.frameName.c_str());

          this->sensorDataList_.push_back(std::move(newSd));
        }
        return true;
      });
}

void LivoxCpuLidarSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  rclcpp::Time simTime(_info.simTime.count());

  for (auto &sd : this->sensorDataList_)
  {
    if (!sd.initialized || sd.aviaInfos.empty())
      continue;

    size_t patternSize = sd.aviaInfos.size();

    struct Point3D { float x, y, z, intensity; };
    std::vector<Point3D> validPoints;
    validPoints.reserve(sd.samplesStep);

    auto poseComp = _ecm.Component<gz::sim::components::Pose>(sd.sensorEntity);
    if (!poseComp)
      continue;

    gz::math::Pose3d pose = poseComp->Data();
    gz::math::Vector3d origin = pose.Pos();
    gz::math::Quaterniond rot = pose.Rot();

    for (int i = 0; i < sd.samplesStep; ++i)
    {
      size_t currentIndex = (sd.patternIndex + i) % patternSize;
      const auto &targetPoint = sd.aviaInfos[currentIndex];

      gz::math::Vector3d dir(
          cos(targetPoint.zenith) * cos(targetPoint.azimuth),
          cos(targetPoint.zenith) * sin(targetPoint.azimuth),
          sin(targetPoint.zenith));
      dir = rot.RotateVector(dir);

      validPoints.push_back(Point3D{
          static_cast<float>((origin.X() + dir.X() * sd.maxDist)),
          static_cast<float>((origin.Y() + dir.Y() * sd.maxDist)),
          static_cast<float>((origin.Z() + dir.Z() * sd.maxDist)),
          100.0f});
    }

    if (!validPoints.empty())
    {
      livox_ros_driver2::msg::CustomMsg livoxMsg;
      livoxMsg.header.stamp = simTime;
      livoxMsg.header.frame_id = sd.frameName;
      livoxMsg.timebase = 0;
      livoxMsg.point_num = validPoints.size();
      livoxMsg.lidar_id = 0;

      for (const auto &pt : validPoints)
      {
        livox_ros_driver2::msg::CustomPoint point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        point.reflectivity = pt.intensity;
        livoxMsg.points.push_back(point);
      }

      sd.customPub->publish(livoxMsg);
    }

    {
      sensor_msgs::msg::PointCloud2 cloudMsg;
      cloudMsg.header.stamp = simTime;
      cloudMsg.header.frame_id = sd.frameName;
      cloudMsg.height = 1;
      cloudMsg.is_dense = true;
      cloudMsg.is_bigendian = false;
      cloudMsg.point_step = 16;
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
          *iterX = pt.x; *iterY = pt.y; *iterZ = pt.z; *iterI = pt.intensity;
          ++iterX; ++iterY; ++iterZ; ++iterI;
        }
      }

      sd.cloud2Pub->publish(cloudMsg);
    }

    sd.patternIndex = (sd.patternIndex + sd.samplesStep) % patternSize;
  }

  rclcpp::spin_some(this->rosNode_);
}

}

// GZ_ADD_PLUGIN(livox_gazebo_garden::LivoxCpuLidarSystem,
//     gz::sim::System,
//     livox_gazebo_garden::LivoxCpuLidarSystem::ISystemConfigure,
//     livox_gazebo_garden::LivoxCpuLidarSystem::ISystemPreUpdate,
//     livox_gazebo_garden::LivoxCpuLidarSystem::ISystemPostUpdate)

// GZ_ADD_PLUGIN_ALIAS(livox_gazebo_garden::LivoxCpuLidarSystem,
//     "livox_gazebo_garden::LivoxCpuLidarSystem")
