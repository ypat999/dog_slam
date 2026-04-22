#ifndef LIVOX_GAZEBO_GARDEN_GPU_LASER_HH_
#define LIVOX_GAZEBO_GARDEN_GPU_LASER_HH_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/sensors/GpuLidarSensor.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace livox_gazebo_garden
{

struct ScanPatternPoint
{
  double time;
  double azimuth;
  double zenith;
};

class LivoxGpuLaserSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  LivoxGpuLaserSystem();
  ~LivoxGpuLaserSystem() override;

  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

private:
  void LoadCsvPattern(const std::string &csv_path);
  std::string ResolvePackageURI(const std::string &uri);

  struct SensorData
  {
    gz::sim::Entity sensorEntity;
    std::shared_ptr<gz::sensors::GpuLidarSensor> gpuLidarSensor;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    std::string topicName;
    std::string frameName;
    std::vector<ScanPatternPoint> scanPattern;
    size_t patternIndex = 0;
    int samplesPerUpdate = 1000;
    int downsample = 1;
    double maxDist = 100.0;
    bool initialized = false;
  };

  std::vector<SensorData> sensorDataList_;
  rclcpp::Node::SharedPtr rosNode_;
  std::unique_ptr<gz::transport::Node> gzNode_;

  std::string robotNamespace_;
  std::string csvFileName_;
};

}
#endif
