export DISPLAY=localhost:10.0
colcon build --symlink-install && source install/setup.bash && ros2 launch slam_offline cartographer_3d.launch.py