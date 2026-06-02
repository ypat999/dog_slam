#!/bin/bash

echo "echo ./kill_play_ros2_bag.sh"

printf "\nstop playing ros2 bag now...\n"
bag_play_pid=$(ps aux | grep 'bag play' | grep python | tr -s ' ' | cut -d ' ' -f 2)

if [ -z "$bag_play_pid" ]; then
  echo "no ros2 bag play script running"
else
  echo "bag_play_pid=$bag_play_pid"
  echo "kill $bag_play_pid"
  kill $bag_play_pid
fi
