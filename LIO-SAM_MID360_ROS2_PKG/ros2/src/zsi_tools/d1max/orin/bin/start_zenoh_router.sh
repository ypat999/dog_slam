#! /bin/bash

# sleep 2
# echo '1' | sudo -S ifconfig enP8p1s0 multicast
# echo '1' | sudo -S route add -net 239.0.0.0 netmask 255.255.255.0 enP8p1s0
# sleep 2

. /opt/runtime/env.bash

ros2 run rmw_zenoh_cpp rmw_zenohd
