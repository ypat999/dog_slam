#!/bin/bash

# LIO-SAM NAV2 Service Installation Script
# This script installs the LIO-SAM NAV2 service for automatic startup

set -e

echo "Installing LIO-SAM NAV2 Service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root or with sudo"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if we're in the correct location
EXPECTED_PATH="/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG"
if [ "$SCRIPT_DIR" != "$EXPECTED_PATH" ]; then
    echo "Warning: Script is running from $SCRIPT_DIR"
    echo "Expected path: $EXPECTED_PATH"
    echo "Please ensure you're in the correct directory."
fi

# Add user to required groups if not already
if ! groups ztl | grep -q "dialout"; then
    echo "Adding ztl user to dialout group..."
    usermod -aG dialout ztl
fi

if ! groups ztl | grep -q "audio"; then
    echo "Adding ztl user to audio group..."
    usermod -aG audio ztl
fi

# Install the service
echo "Installing systemd service..."
cp "$SCRIPT_DIR/lio_sam_nav2.service" /etc/systemd/system/

# Reload systemd and enable service
systemctl daemon-reload
systemctl enable lio_sam_nav2.service

echo "Service installation completed!"
echo "To start the service: sudo systemctl start lio_sam_nav2"
echo "To check status: sudo systemctl status lio_sam_nav2"
echo "To view logs: sudo journalctl -u lio_sam_nav2 -f"
echo ""
echo "Note: You may need to log out and back in for group changes to take effect."