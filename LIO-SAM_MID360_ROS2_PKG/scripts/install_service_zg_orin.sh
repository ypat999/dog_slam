#!/bin/bash

# LIO-SAM NAV2 Service Installation Script
# This script installs the LIO-SAM NAV2 service for automatic startup

set -e

echo "Installing LIO-SAM NAV2 Orin Service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root or with sudo"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if we're in the correct location
EXPECTED_PATH="/home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/scripts"
if [ "$SCRIPT_DIR" != "$EXPECTED_PATH" ]; then
    echo "Warning: Script is running from $SCRIPT_DIR"
    echo "Expected path: $EXPECTED_PATH"
    echo "Please ensure you're in the correct directory."
fi

# Add user to required groups if not already
if ! groups robot | grep -q "dialout"; then
    echo "Adding robot user to dialout group..."
    usermod -aG dialout robot
fi

if ! groups robot | grep -q "audio"; then
    echo "Adding robot user to audio group..."
    usermod -aG audio robot
fi
echo "Installing required packages..."

apt install ros-humble-pointcloud-to-laserscan

echo "Creating symbolic links..."
mkdir /home/robot/rkbot/src/
ln -sd /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/zsi_tools/fast_lio_robosenseAiry/ /home/robot/rkbot/src/fast_lio_robosenseAiry
ln -sd /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/global_config/ /home/robot/rkbot/src/global_config
ln -sd /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/Super-LIO/ /home/robot/rkbot/src/Super-LIO
ln -sd /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/zsi_tools/zg_double_lidar/ /home/robot/rkbot/src/zg_double_lidar

# Install the service
echo "Installing systemd service..."
# cp "$SCRIPT_DIR/lio_sam_buildmap.zg.service" /etc/systemd/system/lio_sam_buildmap.service
# cp "$SCRIPT_DIR/lio_sam_nav2.zg.service" /etc/systemd/system/lio_sam_nav2.service
# cp "$SCRIPT_DIR/auto_buildmap.zg.service" /etc/systemd/system/auto_buildmap.service
cp "$SCRIPT_DIR/zg_pointcloud.service" /etc/systemd/system/zg_pointcloud.service



# Reload systemd and enable service
systemctl daemon-reload

echo "Copying required files..."
cp /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/zsi_tools/d1max/orin/zenoh_router_config.json5 /opt/runtime/config/zenoh_router_config.json5
cp /home/robot/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/zsi_tools/d1max/orin/bin/robot_start_app.sh /opt/runtime/bin/robot_start_app.sh


# systemctl enable lio_sam_nav2.service
# systemctl enable zg_pointcloud.service

echo "Service installation completed!"
echo ""
echo "Note: You may need to log out and back in for group changes to take effect."