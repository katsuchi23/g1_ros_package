# FAST_LIO_LOCALIZATION - ROS2 Humble Migration

This package has been migrated from ROS1 to ROS2 Humble.

## Changes Made

### 1. Package Structure
- **package.xml**: Updated to format 3 with ROS2 dependencies
  - Changed from catkin to ament_cmake
  - Updated dependencies: rclcpp, rclpy, tf2, tf2_ros, livox_ros_driver2, etc.

### 2. C++ Code Migration
- **laserMapping.cpp**: 
  - Replaced `ros::` with `rclcpp::`
  - Updated message types from `sensor_msgs::` to `sensor_msgs::msg::`
  - Changed `tf::` to `tf2::` for transformations
  - Updated parameter handling from `nh.param<>()` to `node->declare_parameter()` and `get_parameter()`
  - Changed time handling from `ros::Time` to `rclcpp::Time`
  - Updated publishers/subscribers to use shared pointers

- **preprocess.cpp/.h**:
  - Updated Livox driver messages from `livox_ros_driver::` to `livox_ros_driver2::msg::`
  - Changed message pointers from `::ConstPtr` to `::SharedPtr`

### 3. Python Scripts Migration
- **global_localization.py**:
  - Changed from Python 2 to Python 3
  - Replaced `rospy` with `rclpy` and Node-based architecture
  - Updated `tf` to `scipy.spatial.transform.Rotation` and `tf2_ros`
  - Changed `thread` to `threading`
  - Updated point cloud handling

- **transform_fusion.py**:
  - Similar ROS2 migration with rclpy
  - Updated TF2 broadcasting

- **publish_initial_pose.py**:
  - Migrated to rclpy
  - Updated quaternion handling

### 4. Launch Files
- Converted XML launch files to Python launch files
- Created `.launch.py` files for different LiDAR types

### 5. Configuration Files
- Updated YAML files to use ROS2 parameter format with `ros__parameters`
- Ensured all values are properly typed (floats, bools)

## Dependencies

### System Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs \
    python3-open3d \
    python3-scipy
```

### Python Dependencies
```bash
pip3 install open3d scipy
```

### Workspace Dependencies
- **livox_ros_driver2**: Make sure you have the ROS2 version of Livox driver in your workspace
- **FAST_LIO** (optional): If you want to generate maps first

## Building

```bash
cd ~/ws_livox
colcon build --packages-select fast_lio_localization --symlink-install
source install/setup.bash
```

## Usage

### 1. Prepare Your Map
First, you need a PCD map file. You can generate one using FAST_LIO:

```bash
# Run FAST_LIO to create a map
ros2 launch fast_lio mapping_avia.launch.py

# After mapping, your PCD file will be saved
```

### 2. Launch Localization

For Livox Avia:
```bash
ros2 launch fast_lio_localization localization_avia.launch.py map:=/path/to/your/map.pcd
```

For Velodyne:
```bash
ros2 launch fast_lio_localization localization_velodyne.launch.py map:=/path/to/your/map.pcd
```

### 3. Set Initial Pose

You can set the initial pose in two ways:

**Method 1**: Use RViz2's "2D Pose Estimate" tool

**Method 2**: Use the script:
```bash
ros2 run fast_lio_localization publish_initial_pose.py <x> <y> <z> <yaw> <pitch> <roll>
```

Example:
```bash
ros2 run fast_lio_localization publish_initial_pose.py 0.0 0.0 0.0 0.0 0.0 0.0
```

## Topics

### Subscribed Topics
- `/livox/lidar` or `/livox/points`: LiDAR point cloud
- `/livox/imu`: IMU data
- `/map`: Global map (PCD format)
- `/initialpose`: Initial pose for localization

### Published Topics
- `/Odometry`: Odometry in camera_init frame
- `/localization`: Global localization result in map frame
- `/cloud_registered`: Registered point cloud
- `/path`: Robot trajectory
- `/cur_scan_in_map`: Current scan transformed to map frame
- `/submap`: Submap in FOV
- `/map_to_odom`: Transform from map to odom frame

### TF Frames
- `map` -> `camera_init` -> `body`

## Configuration

Edit the config files in `config/` directory:
- `avia.yaml`: For Livox Avia
- `velodyne.yaml`: For Velodyne
- `ouster64.yaml`: For Ouster
- `horizon.yaml`: For Livox Horizon

Key parameters:
- `common.lid_topic`: LiDAR topic name
- `common.imu_topic`: IMU topic name
- `preprocess.lidar_type`: 1=Livox, 2=Velodyne, 3=Ouster
- `mapping.fov_degree`: Field of view
- `mapping.det_range`: Detection range
- `mapping.extrinsic_T/R`: Extrinsic calibration between LiDAR and IMU

## Troubleshooting

### Issue: "livox_ros_driver2 not found"
Make sure you have the ROS2 version of Livox driver:
```bash
cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
colcon build --packages-select livox_ros_driver2
```

### Issue: Python import errors
Install required Python packages:
```bash
pip3 install open3d scipy numpy
```

### Issue: Can't find ROS2 numpy utilities
If `ros2_numpy` is not available, the code uses sensor_msgs_py.point_cloud2 instead.

### Issue: TF lookup errors
Make sure all nodes are running and publishing their transforms. Check with:
```bash
ros2 run tf2_ros tf2_echo map body
```

## Notes

- The original ROS1 code has been backed up with `_ros1_backup` suffix
- Make sure your map file is in the correct frame (usually `camera_init` or `map`)
- Localization frequency and thresholds can be adjusted in the Python scripts
- For best results, provide a good initial pose estimate

## License

BSD License (same as original FAST_LIO)

## References

- Original FAST_LIO: https://github.com/hku-mars/FAST_LIO
- Livox ROS Driver 2: https://github.com/Livox-SDK/livox_ros_driver2
