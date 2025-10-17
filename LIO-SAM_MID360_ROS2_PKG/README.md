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

### Build
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

### Relocalization
sudo apt install ros-humble-ndt-omp
sudo apt install ros-humble-pcl-ros


### Convert to Occupancy Grid
sudo apt install ros-humble-octomap ros-humble-octomap-msgs

<!-- sudo apt install ros-humble-pointcloud-to-laserscan -->

<!-- sudo apt install ros-humble-pcl-ros ros-humble-nav2-costmap-2d -->
