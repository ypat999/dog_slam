#!/usr/bin/env bash
set -e

FILE="$1"

if [[ -z "$FILE" ]]; then
  echo "Usage: $0 file.yaml"
  exit 1
fi

sed -i \
  -e 's|^/image_publisher/camera_back/compressed:|/rear_camera/image_compressed:|' \
  -e 's|^/image_publisher/camera_front/compressed:|/front_camera/image_compressed:|' \
  -e 's|^/rslidar_points_head:|/front_lidar:|' \
  -e 's|^/rslidar_points_tail:|/rear_lidar:|' \
  "$FILE"

