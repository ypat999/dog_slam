#!/bin/bash

# Robot runtime startup script (early initialization)
#
# What this script does:
# 1) Clean up any leftover upgrade/OTA directory that could affect the current run.
# 2) Tune SoC thermal trip points and set CPUFreq governors (bias toward performance / lower jitter).
# 3) Block until system time is synchronized (NTP/PTP/external time source), then continue.
# 4) Ensure device.json migrated to new path if needed
#
# Note: this script typically requires root privileges to write under /sys.
# If it fails, check permissions, kernel configuration, and whether the sysfs nodes exist.

# --- Configurable paths (allow override via env) ---
DEVICE_JSON_OUTDATED="${DEVICE_JSON_OUTDATED:-/userdata/robot/device.json}"
DEVICE_JSON_NEW="${DEVICE_JSON_NEW:-/userdata/config/device.json}"

# Migrate device.json if needed
# If new path missing, but outdated exists, copy once.
if [ ! -f "$DEVICE_JSON_NEW" ]; then
  if [ -f "$DEVICE_JSON_OUTDATED" ]; then
    mkdir -p "$(dirname "$DEVICE_JSON_NEW")"
    cp -a "$DEVICE_JSON_OUTDATED" "$DEVICE_JSON_NEW"
  fi
fi

# Cleanup upgrade directory: an interrupted upgrade/OTA may leave stale files behind.
if [ -d /userdata/upgrade ]; then
    rm -rf /userdata/upgrade
fi

# Tune thermal_zone0 trip points (unit: millicelsius, m°C)
# trip_point_0_temp: 100000 = 100°C
# trip_point_1_temp:  95000 =  95°C
# trip_point_2_temp: 120000 = 120°C
# Exact behavior depends on the thermal zone policy and bound cooling devices.
echo 100000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
echo 95000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
echo 120000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp

# Set CPUFreq governor to "performance": keep higher clocks to reduce latency/jitter.
# policy0/policy4/policy6 typically map to different CPU clusters/core groups (platform-dependent).
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo performance > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
echo performance > /sys/devices/system/cpu/cpufreq/policy6/scaling_governor

# Wait for time synchronization.
# This usually blocks until system time is trustworthy (avoids issues with timestamps/logging/
# certificate validation/distributed communication).
/opt/runtime/bin/robot_wait_time_sync.sh
