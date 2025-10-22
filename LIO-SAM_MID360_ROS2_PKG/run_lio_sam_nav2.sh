echo $DISPLAY
export ROS_DOMAIN_ID=23
export DISPLAY=localhost:10.0
cd /home/ywj/projects/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/
colcon build --packages-select lio_sam && source install/setup.bash && ros2 launch lio_sam lio_sam_nav2.launch.py