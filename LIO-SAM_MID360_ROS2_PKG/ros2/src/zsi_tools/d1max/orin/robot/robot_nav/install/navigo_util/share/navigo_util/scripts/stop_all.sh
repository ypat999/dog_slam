#! /bin/bash

function robot_stop() {
  robot-launch stop mapping localization perception perception-seg nav2
}

robot_stop
