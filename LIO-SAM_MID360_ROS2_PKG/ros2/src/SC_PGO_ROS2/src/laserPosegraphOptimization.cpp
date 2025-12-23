#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
#include <pcl/common/transforms.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/octree/octree_pointcloud_voxelcentroid.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <sensor_msgs/Imu.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include <eigen3/Eigen/Dense>

// #include <ceres/ceres.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0;
double rotaionAccumulated = 1000000.0;

bool isNowKeyFrame = false;

Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init
Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // init pose is zero

std::queue<std::shared_ptr<nav_msgs::msg::Odometry>> odometryBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> fullResBuf;
std::queue<std::shared_ptr<sensor_msgs::msg::NavSatFix>> gpsBuf;
std::queue<std::pair<int, int>> scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(
    new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(
    new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(
    new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
// bool useGPS = false;
sensor_msgs::msg::NavSatFix::SharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;

std::shared_ptr<rclcpp::Node> nh;

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D &p) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
                      gtsam::Point3(p.x, p.y, p.z));
}  // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename) {
  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);
  for (const auto &_pose6d : keyframePoses) {
    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates,
                                      std::string _filename) {
  using namespace gtsam;

  // ref from gtsam's original code "dataset.cpp"
  std::fstream stream(_filename.c_str(), std::fstream::out);

  for (const auto &key_value : _estimates) {
    auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
    if (!p) continue;

    const Pose3 &pose = p->value();

    Point3 t = pose.translation();
    Rot3 R = pose.rotation();
    auto col1 = R.column(1);  // Point3
    auto col2 = R.column(2);  // Point3
    auto col3 = R.column(3);  // Point3

    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
           << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
           << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
           << " " << t.z() << std::endl;
  }
}

void laserOdometryHandler(
    const nav_msgs::msg::Odometry::SharedPtr _laserOdometry) {
  mBuf.lock();
  odometryBuf.push(_laserOdometry);
  mBuf.unlock();
}  // laserOdometryHandler

void laserCloudFullResHandler(
    sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes) {
  mBuf.lock();
  fullResBuf.push(_laserCloudFullRes);
  mBuf.unlock();
}  // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps) {
  if (useGPS) {
    mBuf.lock();
    gpsBuf.push(_gps);
    mBuf.unlock();
  }
}  // gpsHandler

void initNoises(void) {
  gtsam::Vector priorNoiseVector6(6);
  priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

  gtsam::Vector odomNoiseVector6(6);
  // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
  odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

  double loopNoiseScore = 0.5;  // constant is ok...
  gtsam::Vector robustNoiseVector6(
      6);  // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
      loopNoiseScore, loopNoiseScore, loopNoiseScore;
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(
          1),  // optional: replacing Cauchy by DCS or GemanMcClure is okay but
               // Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  double bigNoiseTolerentToXY = 1000000000.0;  // 1e9
  double gpsAltitudeNoiseScore = 250.0;  // if height is misaligned after loop
                                         // clsosing, use this value bigger
  gtsam::Vector robustNoiseVector3(3);   // gps factor has 3 elements (xyz)
  robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
      gpsAltitudeNoiseScore;  // means only caring altitude here. (because
                              // LOAM-like-methods tends to be asymptotically
                              // flyging)
  robustGPSNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(
          1),  // optional: replacing Cauchy by DCS or GemanMcClure is okay but
               // Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}  // initNoises

Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr &_odom) {
  auto tx = _odom->pose.pose.position.x;
  auto ty = _odom->pose.pose.position.y;
  auto tz = _odom->pose.pose.position.z;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  return Pose6D{tx, ty, tz, roll, pitch, yaw};
}  // getOdom

Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2) {
  Eigen::Affine3f SE3_p1 =
      pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
  Eigen::Affine3f SE3_p2 =
      pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
  Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
  Eigen::Affine3f SE3_delta;
  SE3_delta.matrix() = SE3_delta0;
  float dx, dy, dz, droll, dpitch, dyaw;
  pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
  // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll
  // << ", " << dpitch << ", " << dyaw << std::endl;

  return Pose6D{double(abs(dx)),    double(abs(dy)),     double(abs(dz)),
                double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
}  // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(
    const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D &tf) {
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur =
      pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

  int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                            transCur(0, 1) * pointFrom.y +
                            transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                            transCur(1, 1) * pointFrom.y +
                            transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                            transCur(2, 1) * pointFrom.y +
                            transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }

  return cloudOut;
}

void pubPath(void) {
  // Publish odom and path
  nav_msgs::msg::Odometry odomAftPGO;
  nav_msgs::msg::Path pathAftPGO;
  pathAftPGO.header.frame_id = "camera_init";
  mKF.lock();
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    const Pose6D &pose_est =
        keyframePosesUpdated.at(node_idx);  // Updated poses

    nav_msgs::msg::Odometry odomAftPGOthis;
    odomAftPGOthis.header.frame_id = "camera_init";
    odomAftPGOthis.child_frame_id = "aft_pgo";
    odomAftPGOthis.header.stamp =
        rclcpp::Time(keyframeTimes.at(node_idx) * 1e9);
    odomAftPGOthis.pose.pose.position.x = pose_est.x;
    odomAftPGOthis.pose.pose.position.y = pose_est.y;
    odomAftPGOthis.pose.pose.position.z = pose_est.z;

    tf2::Quaternion q;
    q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
    odomAftPGOthis.pose.pose.orientation.x = q.x();
    odomAftPGOthis.pose.pose.orientation.y = q.y();
    odomAftPGOthis.pose.pose.orientation.z = q.z();
    odomAftPGOthis.pose.pose.orientation.w = q.w();
    odomAftPGO = odomAftPGOthis;

    geometry_msgs::msg::PoseStamped poseStampAftPGO;
    poseStampAftPGO.header = odomAftPGOthis.header;
    poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

    pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
    pathAftPGO.header.frame_id = "camera_init";
    pathAftPGO.poses.push_back(poseStampAftPGO);
  }
  mKF.unlock();
  pubOdomAftPGO->publish(odomAftPGO);  // Last pose
  pubPathAftPGO->publish(pathAftPGO);  // Poses

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = odomAftPGO.header.stamp;
  transformStamped.header.frame_id = "camera_init";
  transformStamped.child_frame_id = "aft_pgo";
  transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
  transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
  transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
  transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;

  static std::shared_ptr<tf2_ros::TransformBroadcaster> br = nullptr;
  if (!br) {
    br = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  }
  br->sendTransform(transformStamped);
}  // pubPath

void updatePoses(void) {
  mKF.lock();
  for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size());
       node_idx++) {
    Pose6D &p = keyframePosesUpdated[node_idx];
    p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
    p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
    p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
    p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
    p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
    p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
  }
  mKF.unlock();

  mtxRecentPose.lock();
  const gtsam::Pose3 &lastOptimizedPose =
      isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size()) - 1);
  recentOptimizedX = lastOptimizedPose.translation().x();
  recentOptimizedY = lastOptimizedPose.translation().y();

  recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

  mtxRecentPose.unlock();
}  // updatePoses

void runISAM2opt(void) {
  // called when a variable added
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  isamCurrentEstimate = isam->calculateEstimate();
  updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn) {
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(
      transformIn.translation().x(), transformIn.translation().y(),
      transformIn.translation().z(), transformIn.rotation().roll(),
      transformIn.rotation().pitch(), transformIn.rotation().yaw());

  int numberOfCores = 8;  // TODO move to yaml
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom->x +
                            transCur(0, 1) * pointFrom->y +
                            transCur(0, 2) * pointFrom->z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom->x +
                            transCur(1, 1) * pointFrom->y +
                            transCur(1, 2) * pointFrom->z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom->x +
                            transCur(2, 1) * pointFrom->y +
                            transCur(2, 2) * pointFrom->z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom->intensity;
  }
  return cloudOut;
}  // transformPointCloud

void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                                const int &key, const int &submap_size,
                                const int &root_idx) {
  // extract and stacking near keyframes (in global coord)
  nearKeyframes->clear();
  for (int i = -submap_size; i <= submap_size; ++i) {
    int keyNear = key + i;
    if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size())) continue;

    mKF.lock();
    *nearKeyframes += *local2global(keyframeLaserClouds[keyNear],
                                    keyframePosesUpdated[root_idx]);
    mKF.unlock();
  }

  if (nearKeyframes->empty()) return;

  // downsample near keyframes
  pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
  downSizeFilterICP.setInputCloud(nearKeyframes);
  downSizeFilterICP.filter(*cloud_temp);
  *nearKeyframes = *cloud_temp;
}  // loopFindNearKeyframesCloud

std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx,
                                                 int _curr_kf_idx) {
  // parse pointclouds
  int historyKeyframeSearchNum =
      25;  // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every
           // kf gap is 1m
  pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(
      new pcl::PointCloud<PointType>());
  loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0,
                             _loop_kf_idx);  // use same root of loop kf idx
  loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx,
                             historyKeyframeSearchNum, _loop_kf_idx);

  // loop verification
  sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
  pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
  cureKeyframeCloudMsg.header.frame_id = "camera_init";
  pubLoopScanLocal->publish(cureKeyframeCloudMsg);

  sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
  pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
  targetKeyframeCloudMsg.header.frame_id = "camera_init";
  pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(
      150);  // giseop , use a value can cover 2*historyKeyframeSearchNum range
             // in meter
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  // Align pointclouds
  icp.setInputSource(cureKeyframeCloud);
  icp.setInputTarget(targetKeyframeCloud);
  pcl::PointCloud<PointType>::Ptr unused_result(
      new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  float loopFitnessScoreThreshold =
      0.3;  // user parameter but fixed low value is safe.
  if (icp.hasConverged() == false ||
      icp.getFitnessScore() > loopFitnessScoreThreshold) {
    std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore()
              << " > " << loopFitnessScoreThreshold << "). Reject this SC loop."
              << std::endl;
    return std::nullopt;
  } else {
    std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore()
              << " < " << loopFitnessScoreThreshold << "). Add this SC loop."
              << std::endl;
  }

  // Get pose transformation
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionLidarFrame;
  correctionLidarFrame = icp.getFinalTransformation();
  pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch,
                                    yaw);
  gtsam::Pose3 poseFrom =
      Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo =
      Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

  return poseFrom.between(poseTo);
}  // doICPVirtualRelative

template <typename PointT>
void removeNaNAndInfiniteInPlace(typename pcl::PointCloud<PointT>::Ptr &cloud) {
  if (!cloud || cloud->empty()) return;

  // First pass: remove NaNs using PCL’s built-in
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Second pass: remove infinities in-place
  size_t write_idx = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto &pt = cloud->points[i];
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      cloud->points[write_idx++] = pt;
    }
  }

  cloud->points.resize(write_idx);
  cloud->width = static_cast<uint32_t>(write_idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

void process_pg() {
  while (1) {
    while (!odometryBuf.empty() && !fullResBuf.empty()) {
      //
      // pop and check keyframe is or not
      //
      mBuf.lock();
      while (!odometryBuf.empty() &&
             rclcpp::Time(odometryBuf.front()->header.stamp).seconds() <
                 rclcpp::Time(fullResBuf.front()->header.stamp).seconds())
        odometryBuf.pop();
      if (odometryBuf.empty()) {
        mBuf.unlock();
        break;
      }

      // Time equal check
      timeLaserOdometry =
          rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
      timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
      // TODO

      laserCloudFullRes->clear();
      pcl::PointCloud<PointType>::Ptr thisKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
      fullResBuf.pop();

      Pose6D pose_curr = getOdom(odometryBuf.front());
      odometryBuf.pop();

      // find nearest gps
      double eps = 0.1;  // find a gps topioc arrived within eps second
      while (!gpsBuf.empty()) {
        auto thisGPS = gpsBuf.front();
        auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
        if (abs(thisGPSTime - timeLaserOdometry) < eps) {
          currGPS = thisGPS;
          hasGPSforThisKF = true;
          break;
        } else {
          hasGPSforThisKF = false;
        }
        gpsBuf.pop();
      }
      mBuf.unlock();

      //
      // Early reject by counting local delta movement (for equi-spereated kf
      // drop)
      //
      odom_pose_prev = odom_pose_curr;
      odom_pose_curr = pose_curr;
      Pose6D dtf = diffTransformation(
          odom_pose_prev, odom_pose_curr);  // dtf means delta_transform

      double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y +
                                      dtf.z * dtf.z);  // note: absolute value.
      translationAccumulated += delta_translation;
      rotaionAccumulated +=
          (dtf.roll + dtf.pitch + dtf.yaw);  // sum just naive approach.

      if (translationAccumulated > keyframeMeterGap ||
          rotaionAccumulated > keyframeRadGap) {
        isNowKeyFrame = true;
        translationAccumulated = 0.0;  // reset
        rotaionAccumulated = 0.0;      // reset
      } else {
        isNowKeyFrame = false;
      }

      if (!isNowKeyFrame) continue;

      if (!gpsOffsetInitialized) {
        if (hasGPSforThisKF) {  // if the very first frame
          gpsAltitudeInitOffset = currGPS->altitude;
          gpsOffsetInitialized = true;
        }
      }

      //
      // Save data and Add consecutive node
      //
      pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(
          new pcl::PointCloud<PointType>());
      downSizeFilterScancontext.setInputCloud(thisKeyFrame);
      downSizeFilterScancontext.filter(*thisKeyFrameDS);
      removeNaNAndInfiniteInPlace<PointType>(thisKeyFrameDS);

      mKF.lock();
      keyframeLaserClouds.push_back(thisKeyFrameDS);
      keyframePoses.push_back(pose_curr);
      keyframePosesUpdated.push_back(pose_curr);  // init
      keyframeTimes.push_back(timeLaserOdometry);

      scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

      laserCloudMapPGORedraw = true;
      mKF.unlock();

      const int prev_node_idx = keyframePoses.size() - 2;
      const int curr_node_idx =
          keyframePoses.size() -
          1;  // becuase cpp starts with 0 (actually this index could be any
              // number, but for simple implementation, we follow sequential
              // indexing)
      if (!gtSAMgraphMade /* prior node */) {
        const int init_node_idx = 0;
        gtsam::Pose3 poseOrigin =
            Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
        // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
        // gtsam::Point3(0.0, 0.0, 0.0));

        mtxPosegraph.lock();
        {
          // prior factor
          gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
              init_node_idx, poseOrigin, priorNoise));
          initialEstimate.insert(init_node_idx, poseOrigin);
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

        gtSAMgraphMade = true;

        cout << "posegraph prior node " << init_node_idx << " added" << endl;
      } else /* consecutive node (and odom factor) after the prior added */
      {      // == keyframePoses.size() > 1
        gtsam::Pose3 poseFrom =
            Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
        gtsam::Pose3 poseTo =
            Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

        mtxPosegraph.lock();
        {
          // odom factor
          gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
              prev_node_idx, curr_node_idx, poseFrom.between(poseTo),
              odomNoise));

          // gps factor
          if (hasGPSforThisKF) {
            double curr_altitude_offseted =
                currGPS->altitude - gpsAltitudeInitOffset;
            mtxRecentPose.lock();
            gtsam::Point3 gpsConstraint(
                recentOptimizedX, recentOptimizedY,
                curr_altitude_offseted);  // in this example, only adjusting
                                          // altitude (for x and y, very big
                                          // noises are set)
            mtxRecentPose.unlock();
            gtSAMgraph.add(
                gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
            cout << "GPS factor added at node " << curr_node_idx << endl;
          }
          initialEstimate.insert(curr_node_idx, poseTo);
          // runISAM2opt();
        }
        mtxPosegraph.unlock();

        if (curr_node_idx % 100 == 0)
          cout << "posegraph odom node " << curr_node_idx << " added." << endl;
      }
      // if want to print the current graph, use gtSAMgraph.print("\nFactor
      // Graph:\n");

      // save utility
      std::string curr_node_idx_str = padZeros(curr_node_idx);
      pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd",
                                 *thisKeyFrame);   // scan
      pgTimeSaveStream << timeLaser << std::endl;  // path
    }

    // ps.
    // scan context detector is running in another thread (in constant Hz, e.g.,
    // 1 Hz) pub path and point cloud in another thread

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_pg

void performSCLoopClosure(void) {
  if (int(keyframePoses.size()) <
      scManager.NUM_EXCLUDE_RECENT)  // do not try too early
    return;

  auto detectResult =
      scManager.detectLoopClosureID();  // first: nn index, second: yaw diff
  int SCclosestHistoryFrameID = detectResult.first;
  if (SCclosestHistoryFrameID != -1) {
    const int prev_node_idx = SCclosestHistoryFrameID;
    const int curr_node_idx =
        keyframePoses.size() - 1;  // because cpp starts 0 and ends n-1
    cout << "Loop detected! - between " << prev_node_idx << " and "
         << curr_node_idx << "" << endl;

    mBuf.lock();
    scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    // addding actual 6D constraints in the other thread, icp_calculation.
    mBuf.unlock();
  }
}  // performSCLoopClosure

void process_lcd() {
  float loopClosureFrequency = 1.0;  // Hz
  rclcpp::Rate rate(loopClosureFrequency);

  while (rclcpp::ok()) {
    rate.sleep();
    performSCLoopClosure();
    // performRSLoopClosure(); // TODO
  }
}

void process_icp(void) {
  while (1) {
    while (!scLoopICPBuf.empty()) {
      if (scLoopICPBuf.size() > 30) {
        RCLCPP_WARN(
            nh->get_logger(),
            "Too many loop closure candidates to be ICPed is waiting ... "
            "Do process_lcd less frequently (adjust loopClosureFrequency)");
      }

      mBuf.lock();
      std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
      scLoopICPBuf.pop();
      mBuf.unlock();

      const int prev_node_idx = loop_idx_pair.first;
      const int curr_node_idx = loop_idx_pair.second;
      auto relative_pose_optional =
          doICPVirtualRelative(prev_node_idx, curr_node_idx);

      if (relative_pose_optional) {
        gtsam::Pose3 relative_pose = relative_pose_optional.value();
        mtxPosegraph.lock();
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
        // runISAM2opt();
        mtxPosegraph.unlock();
      }
    }

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}  // process_icp

void process_viz_path() {
  float hz = 10.0;
  rclcpp::Rate rate(hz);

  while (rclcpp::ok()) {
    rate.sleep();
    if (recentIdxUpdated > 1) {
      pubPath();
    }
  }
}

void process_isam() {
  float hz = 1.0;
  rclcpp::Rate rate(hz);

  while (rclcpp::ok()) {
    rate.sleep();
    if (gtSAMgraphMade) {
      mtxPosegraph.lock();
      runISAM2opt();
      std::cout << "running isam2 optimization ..." << std::endl;
      mtxPosegraph.unlock();

      saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat);
      saveOdometryVerticesKITTIformat(odomKITTIformat);
    }
  }
}

void pubMap(void) {
  int SKIP_FRAMES = 2;  // sparse map visulalization to save computations
  int counter = 0;

  laserCloudMapPGO->clear();

  mKF.lock();
  // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size());
  // node_idx++) {
  for (int node_idx = 0; node_idx < recentIdxUpdated; node_idx++) {
    if (counter % SKIP_FRAMES == 0) {
      *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx],
                                         keyframePosesUpdated[node_idx]);
    }
    counter++;
  }
  mKF.unlock();

  downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
  downSizeFilterMapPGO.filter(*laserCloudMapPGO);

  sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
  pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
  laserCloudMapPGOMsg.header.frame_id = "camera_init";
  pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

void process_viz_map() {
  float vizmapFrequency = 0.1;  // Hz, so one cycle every 10 seconds
  rclcpp::Rate rate(vizmapFrequency);

  while (rclcpp::ok()) {
    rate.sleep();
    if (recentIdxUpdated > 1) {
      pubMap();
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("laserPGO");

  nh->declare_parameter<std::string>("save_directory", "/");
  save_directory = nh->get_parameter("save_directory").as_string();

  nh->declare_parameter<double>("keyframe_meter_gap", 2.0);
  keyframeMeterGap = nh->get_parameter("keyframe_meter_gap").as_double();

  nh->declare_parameter<double>("keyframe_deg_gap", 10.0);
  keyframeDegGap = nh->get_parameter("keyframe_deg_gap").as_double();

  odomKITTIformat = save_directory + "odom_poses.txt";
  pgTimeSaveStream =
      std::fstream(save_directory + "times.txt", std::fstream::out);
  pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
  pgScansDirectory = save_directory + "Scans/";
  auto unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
  unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

  keyframeRadGap = deg2rad(keyframeDegGap);

  nh->declare_parameter<double>("sc_dist_thres", 0.2);
  scDistThres = nh->get_parameter("sc_dist_thres").as_double();

  nh->declare_parameter<double>(
      "sc_max_radius",
      20.0);  // 80 is recommended for outdoor, and lower (e.g., 20, 40) values
              // are recommended for indoor
  scMaximumRadius = nh->get_parameter("sc_max_radius").as_double();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);
  initNoises();

  scManager.setSCdistThres(scDistThres);
  scManager.setMaximumRadius(scMaximumRadius);

  float filter_size = 0.4;
  downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
  downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

  double mapVizFilterSize;
  nh->declare_parameter<double>("mapviz_filter_size", 0.4);
  mapVizFilterSize = nh->get_parameter("mapviz_filter_size")
                         .as_double();  // pose assignment every k frames
  downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize,
                                   mapVizFilterSize);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      subLaserCloudFullRes;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS;

  subLaserCloudFullRes = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

  subLaserOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 100, laserOdometryHandler);

  subGPS = nh->create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", 100,
                                                                gpsHandler);

  pubOdomAftPGO = nh->create_publisher<nav_msgs::msg::Odometry>(
      "/aft_pgo_odom", rclcpp::QoS(100));
  pubOdomRepubVerifier = nh->create_publisher<nav_msgs::msg::Odometry>(
      "/repub_odom", rclcpp::QoS(100));
  pubPathAftPGO = nh->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path",
                                                            rclcpp::QoS(100));
  pubMapAftPGO = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/aft_pgo_map", rclcpp::QoS(100));

  pubLoopScanLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/loop_scan_local", rclcpp::QoS(100));
  pubLoopSubmapLocal = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/loop_submap_local", rclcpp::QoS(100));

  std::thread posegraph_slam{process_pg};  // pose graph construction
  std::thread lc_detection{process_lcd};   // loop closure detection
  std::thread icp_calculation{
      process_icp};  // loop constraint calculation via icp
  std::thread isam_update{
      process_isam};  // if you want to call less isam2 run (for saving
                      // redundant computations and no real-time visulization is
                      // required), uncommment this and comment all the above
                      // runisam2opt when node is added.

  std::thread viz_map{process_viz_map};  // visualization - map (low frequency
                                         // because it is heavy)
  std::thread viz_path{
      process_viz_path};  // visualization - path (high frequency)

  rclcpp::spin(nh);
  return 0;
}
