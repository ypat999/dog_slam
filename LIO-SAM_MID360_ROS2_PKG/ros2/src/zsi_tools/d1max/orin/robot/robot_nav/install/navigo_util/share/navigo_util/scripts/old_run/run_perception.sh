#! /bin/bash

robot-launch stop ecal2ros
sleep 1
robot-launch start ecal2ros

cd /home/jszr/jszr_workspace && bash install/perception/start_perception.sh
