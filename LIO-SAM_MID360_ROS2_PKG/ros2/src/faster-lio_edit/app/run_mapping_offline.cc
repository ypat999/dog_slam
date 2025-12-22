//
// Created by xiang on 2021/10/9.
//

#include <gflags/gflags.h>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>
#include <memory>

#include "laser_mapping.h"
#include "utils.h"
#include "rclcpp/rclcpp.hpp"

/// run faster-LIO in offline mode

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
// DEFINE_string(bag_file, "/home/xiang/Data/dataset/fast_lio2/avia/2020-09-16-quick-shack.bag", "path to the ros bag");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

// Global flag for exit handling
static volatile bool g_exit_flag = false;

void SigHandle(int sig) {
    g_exit_flag = true;
    RCLCPP_WARN(rclcpp::get_logger("faster_lio"), "catch sig %d", sig);
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("faster_lio_offline");
    
    RCLCPP_WARN(node->get_logger(), "Offline mode is not fully implemented for ROS2 yet. Please use online mode.");

    rclcpp::shutdown();
    return 0;
}