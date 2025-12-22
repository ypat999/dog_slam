//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>
#include <memory>

#include "laser_mapping.h"
#include "rclcpp/rclcpp.hpp"

/// run the lidar mapping in online mode

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

// Global flag for exit handling
static volatile bool g_exit_flag = false;

void SigHandle(int sig) {
    g_exit_flag = true;
    RCLCPP_WARN(rclcpp::get_logger("faster_lio"), "catch sig %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    
    // Filter out ROS-specific arguments that gflags doesn't recognize
    std::vector<char*> filtered_argv;
    filtered_argv.push_back(argv[0]); // Always keep the program name
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // Skip ROS-specific arguments that gflags doesn't recognize
        if (arg != "--ros-args" && arg.substr(0, 13) != "--params-file" && 
            arg.substr(0, 2) != "-r" && arg.substr(0, 2) != "-p" &&
            arg.substr(0, 11) != "__node:=") {
            filtered_argv.push_back(argv[i]);
        }
    }
    
    filtered_argv.push_back(nullptr); // Null-terminate the array
    int filtered_argc = static_cast<int>(filtered_argv.size()) - 1; // Exclude the null terminator
    
    char** filtered_argv_ptr = filtered_argv.data();
    google::ParseCommandLineFlags(&filtered_argc, &filtered_argv_ptr, true);

    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("faster_lio");
    
    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(node);

    signal(SIGINT, SigHandle);
    rclcpp::WallRate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    while (rclcpp::ok() && !g_exit_flag) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        rclcpp::spin_some(node);
        laser_mapping->Run();
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "finishing mapping");
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();
    RCLCPP_INFO(node->get_logger(), "save trajectory to: %s", FLAGS_traj_log_file.c_str());
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    rclcpp::shutdown();
    return 0;
}