export PATH=$PATH:/opt/runtime/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/runtime/lib

source /opt/ros/humble/setup.bash
source /opt/robot/robot-driver/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export ROS_DOMAIN_ID=24
export ROS_LOCALHOST_ONLY=0
export LC_NUMERIC="en_US.UTF-8"
export _colcon_cd_root=/opt/ros/humble/
export RMW_IMPLEMENTATION=rmw_zenoh_cpp #rmw_fastrtps_cpp
export ZENOH_ROUTER_CONFIG_URI=/opt/runtime/config/zenoh_router_config.json5
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{date}] [{severity}] [{name}]: {message}"
export ROBOT_LOG_DIR=/home/robot
# export ROBOT_API_HOST=0.0.0.0
