#!/bin/bash

echo "echo ./kill_record.sh"

printf "\nstop rosbag recording now...\n"
record_rosbag_pid=$(ps aux | grep record | grep python | tr -s ' ' | cut -d ' ' -f 2)

if [ -z "$record_rosbag_pid" ]; then
  echo "no rosbag record script running"
else
  echo "record_rosbag_pid="$record_rosbag_pid
  echo "kill $record_rosbag_pid"
  kill $record_rosbag_pid
fi
