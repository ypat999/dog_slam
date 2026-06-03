#!/bin/bash

PLATFORM="${1:-NX_XG3588}"
FUNCTION_ID="${2:-TrackingWithObstacles}"
LOCALIZATION_SOURCE="${3:-Slam}"

echo "PLATFORM='$PLATFORM'"
echo "FUNCTION_ID='$FUNCTION_ID'"
echo "LOCALIZATION_SOURCE='$LOCALIZATION_SOURCE'"

# Check if tmux is installed
command -v tmux >/dev/null 2>&1 || {
  echo "tmux is not installed. Please install it first."
  exit 1
}

function robot_start() {
  case "$FUNCTION_ID" in
  Tracking)
    robot-launch start perception nav2
    ;;
  TrackingUWB) ;;
  TrackingWithObstacles) ;;
  Patrol | Patrol3D)
    case "$LOCALIZATION_SOURCE" in
    Slam)
      robot-launch start localization perception-seg nav2
      ;;
    Rtk) ;;
    *)
      echo "Unsupported LOCALIZATION_SOURCE: $LOCALIZATION_SOURCE. Use one of {Slam|Rtk}."
      exit 1
      ;;
    esac
    ;;
  Mapping)
    case "$LOCALIZATION_SOURCE" in
    Slam)
      robot-launch start mapping
      ;;
    Rtk) ;;
    *)
      echo "Unsupported LOCALIZATION_SOURCE: $LOCALIZATION_SOURCE. Use one of {Slam|Rtk}."
      exit 1
      ;;
    esac
    ;;
  *)
    echo "Unsupported FUNCTION_ID: $FUNCTION_ID. Use one of {Tracking|TrackingUWB|TrackingWithObstacles|Patrol|Patrol3D|Mapping}."
    exit 1
    ;;
  esac
}

function robot_stop() {
  robot-launch stop mapping localization perception perception-seg nav2
}
function robot_launch() {
  robot_stop
  sleep 1
  robot_start
}

robot_launch
