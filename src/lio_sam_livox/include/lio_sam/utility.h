#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <memory>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

class ParamServer
{
public:

    std::shared_ptr<rclcpp::Node> node;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::shared_ptr<rclcpp::Node> node_ptr)
    {
        node = node_ptr;

        node->declare_parameter<std::string>("robot_id", "roboat");
        node->get_parameter("robot_id", robot_id);

        node->declare_parameter<std::string>("lio_sam.pointCloudTopic", "points_raw");
        node->get_parameter("lio_sam.pointCloudTopic", pointCloudTopic);
        
        node->declare_parameter<std::string>("lio_sam.imuTopic", "imu_correct");
        node->get_parameter("lio_sam.imuTopic", imuTopic);
        
        node->declare_parameter<std::string>("lio_sam.odomTopic", "odometry/imu");
        node->get_parameter("lio_sam.odomTopic", odomTopic);
        
        node->declare_parameter<std::string>("lio_sam.gpsTopic", "odometry/gps");
        node->get_parameter("lio_sam.gpsTopic", gpsTopic);

        node->declare_parameter<std::string>("lio_sam.lidarFrame", "base_link");
        node->get_parameter("lio_sam.lidarFrame", lidarFrame);
        
        node->declare_parameter<std::string>("lio_sam.baselinkFrame", "base_link");
        node->get_parameter("lio_sam.baselinkFrame", baselinkFrame);
        
        node->declare_parameter<std::string>("lio_sam.odometryFrame", "odom");
        node->get_parameter("lio_sam.odometryFrame", odometryFrame);
        
        node->declare_parameter<std::string>("lio_sam.mapFrame", "map");
        node->get_parameter("lio_sam.mapFrame", mapFrame);

        node->declare_parameter<bool>("lio_sam.useImuHeadingInitialization", false);
        node->get_parameter("lio_sam.useImuHeadingInitialization", useImuHeadingInitialization);
        
        node->declare_parameter<bool>("lio_sam.useGpsElevation", false);
        node->get_parameter("lio_sam.useGpsElevation", useGpsElevation);
        
        node->declare_parameter<float>("lio_sam.gpsCovThreshold", 2.0);
        node->get_parameter("lio_sam.gpsCovThreshold", gpsCovThreshold);
        
        node->declare_parameter<float>("lio_sam.poseCovThreshold", 25.0);
        node->get_parameter("lio_sam.poseCovThreshold", poseCovThreshold);

        node->declare_parameter<bool>("lio_sam.savePCD", false);
        node->get_parameter("lio_sam.savePCD", savePCD);
        
        node->declare_parameter<std::string>("lio_sam.savePCDDirectory", "/Downloads/LOAM/");
        node->get_parameter("lio_sam.savePCDDirectory", savePCDDirectory);

        // Sensor type
        std::string sensorStr;
        node->declare_parameter<std::string>("lio_sam.sensor", "");
        node->get_parameter("lio_sam.sensor", sensorStr);
        
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): %s", sensorStr.c_str());
            rclcpp::shutdown();
        }

        node->declare_parameter<int>("lio_sam.N_SCAN", 16);
        node->get_parameter("lio_sam.N_SCAN", N_SCAN);
        
        node->declare_parameter<int>("lio_sam.Horizon_SCAN", 1800);
        node->get_parameter("lio_sam.Horizon_SCAN", Horizon_SCAN);
        
        node->declare_parameter<int>("lio_sam.downsampleRate", 1);
        node->get_parameter("lio_sam.downsampleRate", downsampleRate);
        
        node->declare_parameter<float>("lio_sam.lidarMinRange", 1.0);
        node->get_parameter("lio_sam.lidarMinRange", lidarMinRange);
        
        node->declare_parameter<float>("lio_sam.lidarMaxRange", 1000.0);
        node->get_parameter("lio_sam.lidarMaxRange", lidarMaxRange);

        node->declare_parameter<float>("lio_sam.imuAccNoise", 3.9939570888238808e-03);
        node->get_parameter("lio_sam.imuAccNoise", imuAccNoise);
        
        node->declare_parameter<float>("lio_sam.imuGyrNoise", 1.5636343949698187e-03);
        node->get_parameter("lio_sam.imuGyrNoise", imuGyrNoise);
        
        node->declare_parameter<float>("lio_sam.imuAccBiasN", 6.4356659353532566e-05);
        node->get_parameter("lio_sam.imuAccBiasN", imuAccBiasN);
        
        node->declare_parameter<float>("lio_sam.imuGyrBiasN", 3.5640318696367613e-05);
        node->get_parameter("lio_sam.imuGyrBiasN", imuGyrBiasN);
        
        node->declare_parameter<float>("lio_sam.imuGravity", 9.80511);
        node->get_parameter("lio_sam.imuGravity", imuGravity);
        
        node->declare_parameter<float>("lio_sam.imuRPYWeight", 0.01);
        node->get_parameter("lio_sam.imuRPYWeight", imuRPYWeight);

        // Extrinsics
        node->declare_parameter<std::vector<double>>("lio_sam.extrinsicTrans", {0.0, 0.0, 0.0});
        node->get_parameter("lio_sam.extrinsicTrans", extTransV);
        
        node->declare_parameter<std::vector<double>>("lio_sam.extrinsicRot", std::vector<double>{-1, 0, 0, 0, 1, 0, 0, 0, -1});
        node->get_parameter("lio_sam.extrinsicRot", extRotV);
        
        node->declare_parameter<std::vector<double>>("lio_sam.extrinsicRPY", std::vector<double>{0, -1, 0, 1, 0, 0, 0, 0, 1});
        node->get_parameter("lio_sam.extrinsicRPY", extRPYV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        node->declare_parameter<float>("lio_sam.edgeThreshold", 1.0);
        node->get_parameter("lio_sam.edgeThreshold", edgeThreshold);
        
        node->declare_parameter<float>("lio_sam.surfThreshold", 0.1);
        node->get_parameter("lio_sam.surfThreshold", surfThreshold);
        
        node->declare_parameter<int>("lio_sam.edgeFeatureMinValidNum", 10);
        node->get_parameter("lio_sam.edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        
        node->declare_parameter<int>("lio_sam.surfFeatureMinValidNum", 100);
        node->get_parameter("lio_sam.surfFeatureMinValidNum", surfFeatureMinValidNum);

        // Voxel filter parameters
        node->declare_parameter<float>("lio_sam.odometrySurfLeafSize", 0.4);
        node->get_parameter("lio_sam.odometrySurfLeafSize", odometrySurfLeafSize);
        
        node->declare_parameter<float>("lio_sam.mappingCornerLeafSize", 0.2);
        node->get_parameter("lio_sam.mappingCornerLeafSize", mappingCornerLeafSize);
        
        node->declare_parameter<float>("lio_sam.mappingSurfLeafSize", 0.4);
        node->get_parameter("lio_sam.mappingSurfLeafSize", mappingSurfLeafSize);

        // Robot motion constraint
        node->declare_parameter<float>("lio_sam.z_tollerance", 1000.0);
        node->get_parameter("lio_sam.z_tollerance", z_tollerance);
        
        node->declare_parameter<float>("lio_sam.rotation_tollerance", 1000.0);
        node->get_parameter("lio_sam.rotation_tollerance", rotation_tollerance);

        // CPU parameters
        node->declare_parameter<int>("lio_sam.numberOfCores", 4);
        node->get_parameter("lio_sam.numberOfCores", numberOfCores);
        
        node->declare_parameter<double>("lio_sam.mappingProcessInterval", 0.15);
        node->get_parameter("lio_sam.mappingProcessInterval", mappingProcessInterval);

        // Surrounding map parameters
        node->declare_parameter<float>("lio_sam.surroundingkeyframeAddingDistThreshold", 1.0);
        node->get_parameter("lio_sam.surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        
        node->declare_parameter<float>("lio_sam.surroundingkeyframeAddingAngleThreshold", 0.2);
        node->get_parameter("lio_sam.surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        
        node->declare_parameter<float>("lio_sam.surroundingKeyframeDensity", 2.0);
        node->get_parameter("lio_sam.surroundingKeyframeDensity", surroundingKeyframeDensity);
        
        node->declare_parameter<float>("lio_sam.surroundingKeyframeSearchRadius", 50.0);
        node->get_parameter("lio_sam.surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        // Loop closure parameters
        node->declare_parameter<bool>("lio_sam.loopClosureEnableFlag", true);
        node->get_parameter("lio_sam.loopClosureEnableFlag", loopClosureEnableFlag);
        
        node->declare_parameter<float>("lio_sam.loopClosureFrequency", 1.0);
        node->get_parameter("lio_sam.loopClosureFrequency", loopClosureFrequency);
        
        node->declare_parameter<int>("lio_sam.surroundingKeyframeSize", 50);
        node->get_parameter("lio_sam.surroundingKeyframeSize", surroundingKeyframeSize);
        
        node->declare_parameter<float>("lio_sam.historyKeyframeSearchRadius", 15.0);
        node->get_parameter("lio_sam.historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        
        node->declare_parameter<float>("lio_sam.historyKeyframeSearchTimeDiff", 30.0);
        node->get_parameter("lio_sam.historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        
        node->declare_parameter<int>("lio_sam.historyKeyframeSearchNum", 25);
        node->get_parameter("lio_sam.historyKeyframeSearchNum", historyKeyframeSearchNum);
        
        node->declare_parameter<float>("lio_sam.historyKeyframeFitnessScore", 0.3);
        node->get_parameter("lio_sam.historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        // Global map visualization parameters
        node->declare_parameter<float>("lio_sam.globalMapVisualizationSearchRadius", 1000.0);
        node->get_parameter("lio_sam.globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        
        node->declare_parameter<float>("lio_sam.globalMapVisualizationPoseDensity", 10.0);
        node->get_parameter("lio_sam.globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        
        node->declare_parameter<float>("lio_sam.globalMapVisualizationLeafSize", 1.0);
        node->get_parameter("lio_sam.globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        // Log initialization
        RCLCPP_INFO(node->get_logger(), "LIO-SAM parameters loaded successfully");
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& input)
    {
        sensor_msgs::msg::Imu output = input;
        // rotate acceleration
        Eigen::Vector3d acc(input.linear_acceleration.x, input.linear_acceleration.y, input.linear_acceleration.z);
        acc = extRot * acc;
        output.linear_acceleration.x = acc(0);
        output.linear_acceleration.y = acc(1);
        output.linear_acceleration.z = acc(2);
        // rotate gyroscope
        Eigen::Vector3d gyr(input.angular_velocity.x, input.angular_velocity.y, input.angular_velocity.z);
        gyr = extRot * gyr;
        output.angular_velocity.x = gyr(0);
        output.angular_velocity.y = gyr(1);
        output.angular_velocity.z = gyr(2);
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z);
        Eigen::Quaterniond q_final = extQRPY * q_from;
        output.orientation.x = q_final.x();
        output.orientation.y = q_final.y();
        output.orientation.z = q_final.z();
        output.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            output.orientation.x = 0;
            output.orientation.y = 0;
            output.orientation.z = 0;
            output.orientation.w = 1;
        }

        return output;
    }
};

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg.header.stamp).seconds();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu* thisImuMsg, double* angular_x, double* angular_y, double* angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu* thisImuMsg, double* acc_x, double* acc_y, double* acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu* thisImuMsg, double* roll, double* pitch, double* yaw)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(*roll, *pitch, *yaw);
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
