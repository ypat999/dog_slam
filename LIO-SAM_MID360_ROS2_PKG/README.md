# LIO-SAM_MID360_ROS2_PKG

### Dependency
We require the livox MID360 hardware.
```bash
## LIO-SAM (ros2)
sudo apt install -y ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro

## LIO-SAM (gtsam)
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev
```

### Build (First Time)
delete build/ install/ log/
Run `build_ros2.sh` for the first build. It correctly builds the Livox package.


### Run
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

ros2 launch lio_sam run.launch.py
```

### Save Point Cloud
source install/setup.bash 
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/projects/LOAM/'}"
sudo apt update

// ### Relocalization
// sudo apt install ros-humble-ndt-omp
// sudo apt install ros-humble-pcl-ros


### Convert to Occupancy Grid
sudo apt install ros-humble-octomap ros-humble-octomap-msgs
#### when lio-sam is running, save the map
ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/ywj/projects/map_grid/map --fmt png

### nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-server 
sudo apt-get update && sudo apt-get install -y ros-humble-dwb-critics ros-humble-nav2-dwb-controller ros-humble-nav2-controller ros-humble-nav2-amcl ros-humble-nav2-planner ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager ros-humble-nav2-map-server ros-humble-nav2-waypoint-follower ros-humble-rosbridge-server 


