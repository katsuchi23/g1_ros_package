# pcd2pgm - Point Cloud to Occupancy Grid Converter

Convert 3D PCD (Point Cloud Data) files into 2D PGM (Portable Gray Map) occupancy grid maps for ROS2 Navigation stack.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

<div align="center">

| 3D Point Cloud (PCD) | 2D Occupancy Grid (PGM) |
|:-:|:-:|
|![pcd](.docs/pcd.png)|![pgm](.docs/pgm.png)|

</div>

---

## 📋 Overview

This ROS2 package provides tools to convert 3D point cloud maps (`.pcd` format) into 2D occupancy grid maps (`.pgm` + `.yaml` format) suitable for use with Nav2 and other ROS2 navigation stacks.

### Features

- ✅ Read and process `.pcd` files
- ✅ Pass Through filtering for height-based point selection
- ✅ Radius Outlier filtering for noise reduction
- ✅ Convert to occupancy grid map format
- ✅ Publish processed map to ROS2 topics
- ✅ RViz2 visualization
- ✅ Save maps using `nav2_map_server`

---

## 🔧 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04
- **ROS2**: Humble (or Foxy/Galactic)
- **PCL**: 1.8+ (installed with ROS2)

### Dependencies

```bash
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-nav2-map-server \
    libpcl-dev
```

---

## 📥 Installation

### 1. Create Workspace

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/LihanChen2004/pcd2pgm.git
```

### 3. Install Dependencies

```bash
cd ~/nav_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 4. Build

```bash
cd ~/nav_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 🚀 Usage

### 1. Configure Parameters

Edit the configuration file: `config/pcd2pgm.yaml`

```yaml
pcd2pgm:
  ros__parameters:
    # Input PCD file path
    pcd_file: /path/to/your/map.pcd
    
    # Coordinate transformation from odometry to lidar
    # [x, y, z, roll, pitch, yaw]
    odom_to_lidar_odom: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Filtering options
    flag_pass_through: true           # Enable pass-through filter
    thre_z_min: 0.1                   # Minimum Z height (meters)
    thre_z_max: 2.0                   # Maximum Z height (meters)
    
    # Outlier removal
    thre_radius: 0.1                  # Search radius for outlier removal
    thres_point_count: 10             # Minimum points in radius
    
    # Map parameters
    map_resolution: 0.05              # Grid resolution (meters/pixel)
    map_topic_name: map               # Published map topic name
```

### 2. Launch Conversion

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

This will:
- Load the PCD file
- Apply filters
- Convert to occupancy grid
- Display in RViz2
- Publish on `/map` topic

### 3. Save the Map

In a new terminal:

```bash
# Save map to files (creates .pgm and .yaml)
ros2 run nav2_map_server map_saver_cli -f my_navigation_map

# The output will be:
# - my_navigation_map.pgm (image file)
# - my_navigation_map.yaml (metadata file)
```

---

## ⚙️ Configuration Parameters

### File Paths

| Parameter | Type | Description |
|-----------|------|-------------|
| `pcd_file` | string | Absolute path to input PCD file |

### Transform Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `odom_to_lidar_odom` | array[6] | Transform from odometry frame to lidar frame<br>[x, y, z, roll, pitch, yaw] |

### Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `flag_pass_through` | bool | `false` | Enable Z-axis height filtering |
| `thre_z_min` | float | `0.1` | Minimum Z height to keep (meters) |
| `thre_z_max` | float | `2.0` | Maximum Z height to keep (meters) |
| `thre_radius` | float | `0.1` | Radius for outlier removal (meters) |
| `thres_point_count` | int | `10` | Min points within radius to keep point |

### Map Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_resolution` | float | `0.05` | Grid cell size (meters per pixel) |
| `map_topic_name` | string | `map` | ROS2 topic name for published map |

---

## 📊 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D occupancy grid map |
| `/filtered_cloud` | `sensor_msgs/msg/PointCloud2` | Filtered point cloud (visualization) |

---

## 🎯 Typical Workflow

### Complete Mapping Pipeline

```bash
# Step 1: Create 3D map using FAST_LIO
ros2 launch fast_lio mapping_mid360.launch.py

# Step 2: Convert PCD to 2D navigation map
# (Edit pcd2pgm.yaml to point to your PCD file)
ros2 launch pcd2pgm pcd2pgm_launch.py

# Step 3: Save the map
ros2 run nav2_map_server map_saver_cli -f my_map

# Step 4: Use with Nav2
ros2 launch nav2_bringup navigation_launch.py map:=my_map.yaml
```

---

## 🔧 Tips and Tricks

### Adjusting Height Range

For indoor navigation (robots on floor):
```yaml
thre_z_min: 0.1    # Just above ground
thre_z_max: 2.0    # Below ceiling
```

For outdoor navigation:
```yaml
thre_z_min: -0.5   # Handle uneven terrain
thre_z_max: 3.0    # Include tall obstacles
```

### Map Resolution

Higher resolution = larger file size, more detail:
```yaml
map_resolution: 0.02  # Fine detail (2cm per pixel)
map_resolution: 0.05  # Standard (5cm per pixel)
map_resolution: 0.10  # Coarse (10cm per pixel)
```

### Noise Reduction

If map has too much noise:
```yaml
thre_radius: 0.15        # Increase search radius
thres_point_count: 15    # Require more nearby points
```

If map is too sparse:
```yaml
thre_radius: 0.08        # Decrease search radius
thres_point_count: 5     # Require fewer nearby points
```

---

## 🐛 Troubleshooting

### Issue: Map is empty or has holes

**Cause**: Height filtering is too restrictive

**Solution**: Adjust Z thresholds in config:
```yaml
flag_pass_through: false  # Temporarily disable to check
# OR
thre_z_min: -1.0         # Expand range
thre_z_max: 5.0
```

### Issue: Map has too much noise

**Cause**: Outlier filter is too permissive

**Solution**: Increase outlier removal strictness:
```yaml
thre_radius: 0.15
thres_point_count: 20
```

### Issue: PCD file not found

**Cause**: Incorrect file path

**Solution**: Use absolute path:
```yaml
pcd_file: /home/your_username/lidar_ws/src/FAST_LIO/PCD/scans.pcd
```

### Issue: Map is too large

**Cause**: Resolution is too fine

**Solution**: Increase resolution value:
```yaml
map_resolution: 0.10  # 10cm per pixel instead of 5cm
```

---

## 📚 Related Packages

### Mapping
- [FAST_LIO](../FAST_LIO) - Create 3D PCD maps
- [FAST_LIO_LOCALIZATION](../FAST_LIO_LOCALIZATION) - Localize in PCD maps

### Navigation
- [Nav2](https://navigation.ros.org/) - ROS2 navigation stack
- [nav2_map_server](https://navigation.ros.org/configuration/packages/configuring-map-server.html) - Map serving

### Visualization
- [RViz2](https://github.com/ros2/rviz) - 3D visualization
- [CloudCompare](https://www.danielgm.net/cc/) - Point cloud editing

---

## 🤝 Contributing

Contributions are welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## 📝 License

MIT License - see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

Original package by [LihanChen2004](https://github.com/LihanChen2004)

---

## 📧 Support

- **Issues**: [GitHub Issues](https://github.com/LihanChen2004/pcd2pgm/issues)
- **Documentation**: This README

---

**Convert your 3D maps to 2D navigation grids with ease! 🗺️**
