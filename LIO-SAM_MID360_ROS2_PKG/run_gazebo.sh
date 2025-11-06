#!/bin/bash

# LIO-SAM Gazebo Simulation Launch Script

echo "Starting LIO-SAM Gazebo Simulation..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ./ros2/install/setup.bash

# Kill any existing Gazebo processes
echo "Cleaning up existing Gazebo processes..."
pkill -f gazebo || true
sleep 2

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/ros2/src/LIO-SAM_MID360_ROS2_DOG/models

# Launch Gazebo simulation
echo "Launching Gazebo simulation..."
ros2 launch lio_sam gazebo_simulation.launch.py

echo "Gazebo simulation started. Use Ctrl+C to stop."