echo $DISPLAY
export ROS_DOMAIN_ID=27
#export DISPLAY=localhost:10.0
cd ./ros2/
# colcon build --symlink-install --packages-select lio_sam --executor sequential --parallel-workers 2 && source install/setup.bash && ros2 launch lio_sam lio_sam_nav2.launch.py ns:=/
colcon build --symlink-install --packages-select lio_sam --executor sequential  && source install/setup.bash && ros2 launch lio_sam lio_sam_nav2.launch.py ns:=/
