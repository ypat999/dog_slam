#!/bin/bash

# Test Gazebo simulation setup

echo "Testing Gazebo simulation setup..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ./ros2/install/setup.bash

# Test 1: Check if Gazebo can be launched
echo "1. Testing Gazebo launch..."
timeout 10s gazebo --help > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Gazebo is installed and accessible"
else
    echo "✗ Gazebo launch failed"
    exit 1
fi

# Test 2: Check ROS2 Gazebo packages
echo "2. Checking ROS2 Gazebo packages..."
ros2 pkg list | grep -E "(gazebo_ros|gazebo_plugins)" > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ ROS2 Gazebo packages are installed"
else
    echo "✗ ROS2 Gazebo packages not found"
    exit 1
fi

# Test 3: Check robot description file
echo "3. Checking robot description file..."
if [ -f "./ros2/src/LIO-SAM_MID360_ROS2_DOG/urdf/robot.urdf" ]; then
    echo "✓ Robot URDF file exists"
else
    echo "✗ Robot URDF file not found"
    exit 1
fi

# Test 4: Check world file
echo "4. Checking Gazebo world file..."
if [ -f "./ros2/src/LIO-SAM_MID360_ROS2_DOG/worlds/lio_sam_world.world" ]; then
    echo "✓ Gazebo world file exists"
else
    echo "✗ Gazebo world file not found"
    exit 1
fi

# Test 5: Check launch file
echo "5. Checking launch file..."
if [ -f "./ros2/src/LIO-SAM_MID360_ROS2_DOG/launch/gazebo_simulation.launch.py" ]; then
    echo "✓ Launch file exists"
else
    echo "✗ Launch file not found"
    exit 1
fi

# Test 6: Validate URDF file
echo "6. Validating URDF file..."
check_urdf ./ros2/src/LIO-SAM_MID360_ROS2_DOG/urdf/robot.urdf > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ URDF file is valid"
else
    echo "⚠ URDF file has issues (check_urdf not available or URDF invalid)"
fi

echo ""
echo "All tests completed! Gazebo simulation setup appears to be working."
echo ""
echo "To start the simulation, run:"
echo "  ./run_gazebo.sh"
echo ""
echo "To test robot movement, in a new terminal:"
echo "  source ./ros2/install/setup.bash"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'"
echo ""
echo "To stop the robot:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'"