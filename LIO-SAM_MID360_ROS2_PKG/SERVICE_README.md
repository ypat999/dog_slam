# LIO-SAM NAV2 System Service

This directory contains files to set up LIO-SAM NAV2 as a system service that automatically starts on boot.

## Files Created

- `lio_sam_nav2.service` - Systemd service definition file
- `install_service.sh` - Installation script to set up the service
- `SERVICE_README.md` - This documentation file

## Installation

1. **Navigate to your project directory:**
   ```bash
   cd /home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG
   ```

2. **Run the installation script as root:**
   ```bash
   sudo ./install_service.sh
   ```

3. **Start the service:**
   ```bash
   sudo systemctl start lio_sam_nav2
   ```

4. **Enable automatic startup:**
   ```bash
   sudo systemctl enable lio_sam_nav2
   ```

## Service Management

### Check Service Status
```bash
sudo systemctl status lio_sam_nav2
```

### View Logs
```bash
# View real-time logs
sudo journalctl -u lio_sam_nav2 -f

# View recent logs
sudo journalctl -u lio_sam_nav2 -n 100
```

### Stop Service
```bash
sudo systemctl stop lio_sam_nav2
```

### Restart Service
```bash
sudo systemctl restart lio_sam_nav2
```

### Disable Automatic Startup
```bash
sudo systemctl disable lio_sam_nav2
```

## Service Configuration

The service runs as the `ztl` user and includes:
- Automatic restart on failure (10-second delay)
- Proper ROS_DOMAIN_ID environment variable (27)
- Display configuration for GUI components
- Logging to systemd journal
- User groups: dialout, audio (for hardware access)

## Troubleshooting

1. **Service fails to start:**
   - Check logs: `sudo journalctl -u lio_sam_nav2 -n 50`
   - Verify ROS2 installation: `ros2 --version`
   - Check if required packages are installed

2. **Permission issues:**
   - Ensure the ztl user has proper group memberships (dialout, audio)
   - Check file permissions in `/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/`
   - You may need to log out and back in for group changes to take effect

3. **Network issues:**
   - Verify ROS_DOMAIN_ID matches other ROS2 nodes
   - Check network connectivity

4. **Display issues:**
   - Ensure DISPLAY environment variable is set correctly
   - For remote access, you may need to adjust DISPLAY=:0

## Manual Testing

Before using the service, you can test the launch command manually:
```bash
cd /home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/
colcon build --symlink-install --packages-select lio_sam
source install/setup.bash
ros2 launch lio_sam lio_sam_nav2.launch.py ns:=/
```