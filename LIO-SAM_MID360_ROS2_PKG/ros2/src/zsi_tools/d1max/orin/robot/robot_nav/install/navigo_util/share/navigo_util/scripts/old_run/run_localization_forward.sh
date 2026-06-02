#! /bin/bash

source /opt/ros/humble/setup.bash
source /home/jszr/jszr_workspace/install/setup.bash
export ROS_DOMAIN_ID=24
export ROS_DISCOVERY_SERVER=192.168.133.1:20000

ip="192.168.168.100"
interface=$(ifconfig | grep -B1 "$ip" | head -n 1 | awk '{print $1}' | sed 's/://')
echo "Add network route to $interface"
echo "1" | sudo -S route add -net 239.0.0.0 netmask 255.255.255.0 "$interface"

ros2 launch pub_tf pub_tf.launch.py platform:=NX_XG3588 tf_type:=localization_tf mc_controller_type:=RL_TRACK_VELOCITY
