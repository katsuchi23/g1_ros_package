# FAST_LIO_LOCALIZATION - Mid360 Usage Guide

## Quick Start with Rosbag

### 1. Launch Localization with Rosbag Playback

```bash
# Source the workspace
source /home/rey/ws_livox/install/setup.bash

# Launch with default settings (includes map loading and rosbag playback)
ros2 launch fast_lio_localization localization_mid360.launch.py use_bag:=true

# Or without RViz
ros2 launch fast_lio_localization localization_mid360.launch.py use_bag:=true rviz:=false
```

### 2. Launch Localization WITHOUT Rosbag (for live LiDAR)

```bash
# When your Mid360 LiDAR is connected
ros2 launch fast_lio_localization localization_mid360.launch.py
```

### 3. Custom Launch Options

```bash
# Use a different map file
ros2 launch fast_lio_localization localization_mid360.launch.py \
  map:=/path/to/your/map.pcd

# Use a different rosbag
ros2 launch fast_lio_localization localization_mid360.launch.py \
  use_bag:=true \
  bag_path:=/path/to/your/rosbag2_folder

# Change playback speed (0.5 = half speed, 2.0 = double speed)
ros2 launch fast_lio_localization localization_mid360.launch.py \
  use_bag:=true \
  bag_rate:=0.5
```

## Default Paths

- **Map File**: `/home/rey/ws_livox/src/FAST_LIO_LOCALIZATION/PCD/mapping.pcd`
- **Rosbag**: `/home/rey/ws_livox/src/FAST_LIO_LOCALIZATION/rosbag2_2025_09_08-15_50_47`
- **Config**: Mid360 LiDAR configuration

## Expected Topics

The system subscribes to:
- `/livox/lidar` - Point cloud from Mid360
- `/livox/imu` - IMU data from Mid360

The system publishes:
- `/Odometry` - Current odometry estimate
- `/map` - The loaded PCD map
- `/cloud_registered` - Registered point cloud
- `/path` - Robot trajectory

## Setting Initial Pose

You can set the initial pose using the "2D Pose Estimate" tool in RViz2 or by running:

```bash
ros2 run fast_lio_localization publish_initial_pose.py
```

## Troubleshooting

### Map not loading
Check that the map file exists:
```bash
ls -lh /home/rey/ws_livox/src/FAST_LIO_LOCALIZATION/PCD/mapping.pcd
```

### Rosbag not playing
Check that the rosbag exists:
```bash
ls -lh /home/rey/ws_livox/src/FAST_LIO_LOCALIZATION/rosbag2_2025_09_08-15_50_47
```

Verify rosbag topics:
```bash
ros2 bag info /home/rey/ws_livox/src/FAST_LIO_LOCALIZATION/rosbag2_2025_09_08-15_50_47
```

### No point cloud visualization
Make sure your RViz display settings match the published topics. The localization.rviz file has been updated for ROS2.

## Configuration Files

- **Mid360 Config**: `config/mid360.yaml`
- **RViz Config**: `rviz_cfg/localization.rviz`
- **Launch File**: `launch/localization_mid360.launch.py`

## Notes

- The system uses ROS2 sim time when playing rosbags (`--clock` flag)
- Map is published at 5 Hz
- Point filter is set to 3 for Mid360 (balance between accuracy and performance)
- FOV is 360 degrees with 100m detection range
