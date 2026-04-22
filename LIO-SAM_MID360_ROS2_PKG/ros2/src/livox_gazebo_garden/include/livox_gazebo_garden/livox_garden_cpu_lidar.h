#ifndef LIVOX_GAZEBO_GARDEN_CPU_LIDAR_HH_
#define LIVOX_GAZEBO_GARDEN_CPU_LIDAR_HH_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace livox_gazebo_garden
{

struct AviaRotateInfo
{
  double time;
  double azimuth;
  double zenith;
};

class LivoxCpuLidarSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  LivoxCpuLidarSystem();
  ~LivoxCpuLidarSystem() override;

  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

private:
  bool ReadCsvFile(const std::string &file_name,
      std::vector<std::vector<double>> &datas);
  void ConvertDataToRotateInfo(
      const std::vector<std::vector<double>> &datas,
      std::vector<AviaRotateInfo> &avia_infos);
  std::string ResolvePackageURI(const std::string &uri);

  struct SensorData
  {
    gz::sim::Entity sensorEntity;
    std::string topicName;
    std::string frameName;
    int xferFormat = 1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2Pub;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPub;
    std::vector<AviaRotateInfo> aviaInfos;
    size_t maxPointSize = 0;
    size_t patternIndex = 0;
    int samplesStep = 1000;
    int downSample = 1;
    double minDist = 0.0;
    double maxDist = 100.0;
    bool initialized = false;
  };

  std::vector<SensorData> sensorDataList_;
  rclcpp::Node::SharedPtr rosNode_;
  std::unique_ptr<gz::transport::Node> gzNode_;
};

}
#endif
