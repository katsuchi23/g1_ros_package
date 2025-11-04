# FAST-LIO (ROS2)

> **ROS2 Fork**: Maintained by [Ericsii](https://github.com/Ericsii)

**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter for robust navigation in fast-motion, noisy, or cluttered environments.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-GPLv2-green)](LICENSE)

<div align="center">
<img src="doc/real_experiment2.gif" width=49% />
<img src="doc/ulhkwh_fastlio.gif" width=49% />
</div>

**Related Video:** [FAST-LIO2](https://youtu.be/2OvjGnxszf8) | [FAST-LIO1](https://youtu.be/iYCY6T79oNU)

---

## ✨ Features

### FAST-LIO 2.0 Key Features

1. **Incremental Mapping** using [ikd-Tree](https://github.com/hku-mars/ikd-Tree)
   - Faster speed (>100Hz LiDAR rate)
   - Efficient 3D KD-Tree operations
   
2. **Direct Odometry** on Raw Points
   - No feature extraction required
   - Better accuracy
   - Scan-to-map matching
   
3. **Multi-LiDAR Support**
   - Spinning LiDARs: Velodyne, Ouster
   - Solid-state LiDARs: Livox Avia, Horizon, Mid-70, Mid-360
   - Easy extension to new sensors
   
4. **External IMU Support**
   - Works with external IMU modules
   
5. **ARM Platform Support**
   - Khadas VIM3
   - Nvidia TX2
   - Raspberry Pi 4B (8GB RAM)

<div align="center">
<img src="doc/overview_fastlio2.svg" width=95% />
</div>

---

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Supported Hardware](#supported-hardware)
- [Publications](#publications)

---

## 🔧 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 (or 20.04)
- **ROS2**: Humble, Foxy, or Galactic
- **Memory**: >= 4GB RAM (8GB recommended)
- **CPU**: Multi-core processor

### Dependencies

#### ROS2 Installation
```bash
# ROS2 Humble (recommended)
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-eigen
```

#### PCL and Eigen
```bash
# Default versions from apt are sufficient
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev
```

**Note**: The default PCL (1.12) and Eigen (3.4) from apt are sufficient for FAST-LIO.

#### Livox ROS Driver 2

**Required** for Livox LiDARs:

```bash
cd ~/lidar_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

Alternative (modified version with standard units):
```bash
git clone https://github.com/Ericsii/livox_ros_driver2.git -b feature/use-standard-unit
```

---

## 📥 Installation

### 1. Create Workspace

```bash
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive FAST_LIO
cd FAST_LIO
git submodule update --init --recursive
```

### 3. Install Dependencies

```bash
cd ~/lidar_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
cd ~/lidar_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/lidar_ws/install/setup.bash" >> ~/.bashrc
```

**Important**: Make sure `livox_ros_driver2` is built and sourced before building FAST_LIO.

### Optional: Custom PCL Build

If you need a custom PCL build:
```bash
export PCL_ROOT=/path/to/custom/pcl
```

---

## 🚀 Usage

### 1. Launch with Different LiDARs

#### Livox Mid-360
```bash
ros2 launch fast_lio mapping_mid360.launch.py
```

#### Livox Avia
```bash
ros2 launch fast_lio mapping_avia.launch.py
```

#### Livox Horizon
```bash
ros2 launch fast_lio mapping_horizon.launch.py
```

#### Velodyne VLP-16
```bash
ros2 launch fast_lio mapping_velodyne.launch.py
```

#### Ouster OS1-64
```bash
ros2 launch fast_lio mapping_ouster64.launch.py
```

### 2. With Rosbag

```bash
# Play rosbag in another terminal
ros2 bag play your_data.db3 --clock
```

### 3. Save Map

The map is automatically saved when:
- You stop the node (Ctrl+C)
- Map saving is enabled in config: `pcd_save_enable: true`

Default save location: `PCD/` directory in the package

Adjust save interval in config:
```yaml
pcd_save:
  interval: -1  # -1 = save all frames in one file
                # N > 0 = save every N frames
```

---

## ⚙️ Configuration

### Config Files

Located in `config/` directory:
- `mid360.yaml` - Livox Mid-360
- `avia.yaml` - Livox Avia
- `horizon.yaml` - Livox Horizon
- `velodyne.yaml` - Velodyne LiDAR
- `ouster64.yaml` - Ouster LiDAR

### Key Parameters

```yaml
/**:
  ros__parameters:
    # Feature extraction
    feature_extract_enable: false  # false = use all points (FAST-LIO2 mode)
    point_filter_num: 3            # Downsample factor
    
    # Topics
    common:
      lid_topic: "/livox/lidar"    # LiDAR topic
      imu_topic: "/livox/imu"      # IMU topic
      time_sync_en: false          # External time sync
      
    # LiDAR type
    preprocess:
      lidar_type: 1                # 1=Livox, 2=Velodyne, 3=Ouster
      scan_line: 4                 # Number of scan lines
      blind: 0.5                   # Minimum range (meters)
      
    # Mapping
    mapping:
      fov_degree: 360.0            # Field of view
      det_range: 100.0             # Detection range
      extrinsic_est_en: true       # Auto calibrate IMU-LiDAR
      
      # IMU-LiDAR Extrinsics (if known)
      extrinsic_T: [0.0, 0.0, 0.0]           # Translation [x,y,z]
      extrinsic_R: [1., 0., 0.,              # Rotation matrix
                    0., 1., 0.,
                    0., 0., 1.]
      
    # Publishing
    publish:
      path_en: true                # Publish trajectory
      scan_publish_en: true        # Publish scan
      dense_publish_en: false      # Publish dense cloud
      
    # Map saving
    pcd_save:
      pcd_save_en: true            # Enable map saving
      interval: -1                 # Save interval (-1=all)
```

### LiDAR Type Reference

| Value | LiDAR Type | Examples |
|-------|------------|----------|
| 1 | Livox | Avia, Horizon, Mid-70, Mid-360 |
| 2 | Velodyne | VLP-16, VLP-32, HDL-64 |
| 3 | Ouster | OS0, OS1, OS2 |
| 4 | Custom | Other PointCloud2 sources |

---

## 🖥️ Supported Hardware

### LiDAR Sensors

| Sensor | Status | Config File |
|--------|--------|-------------|
| Livox Mid-360 | ✅ Tested | mid360.yaml |
| Livox Avia | ✅ Tested | avia.yaml |
| Livox Horizon | ✅ Tested | horizon.yaml |
| Livox Mid-70 | ✅ Compatible | avia.yaml (adjust) |
| Velodyne VLP-16 | ✅ Tested | velodyne.yaml |
| Velodyne VLP-32 | ✅ Compatible | velodyne.yaml |
| Ouster OS1-64 | ✅ Tested | ouster64.yaml |
| Ouster OS0/OS2 | ✅ Compatible | ouster64.yaml |

### Computing Platforms

| Platform | RAM | Status | Notes |
|----------|-----|--------|-------|
| PC (x86_64) | 8GB+ | ✅ Recommended | Best performance |
| Nvidia Jetson AGX | 32GB | ✅ Excellent | High performance |
| Nvidia TX2 | 8GB | ✅ Good | Reduce point_filter_num |
| Raspberry Pi 4B | 8GB | ✅ Works | Reduce parameters |
| Khadas VIM3 | 4GB | ⚠️ Limited | Minimal config |

---

## 📊 Topics and Services

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox point cloud |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU data |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/Odometry` | `nav_msgs/msg/Odometry` | Robot odometry |
| `/path` | `nav_msgs/msg/Path` | Robot trajectory |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered points |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Body frame cloud |

### TF Frames
```
map
 └─ camera_init
     └─ body (IMU frame)
         └─ lidar_link
```

---

## 🐛 Troubleshooting

### Build Issues

**Error: livox_ros_driver2 not found**
```bash
# Build livox driver first
cd ~/lidar_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
colcon build --packages-select fast_lio
```

**Error: PCL not found**
```bash
sudo apt install libpcl-dev
```

### Runtime Issues

**No point cloud received**
```bash
# Check if livox driver is running
ros2 topic list | grep livox
ros2 topic hz /livox/lidar

# Verify livox driver configuration
```

**Poor odometry**
- Check IMU-LiDAR extrinsics in config
- Enable extrinsic estimation: `extrinsic_est_en: true`
- Increase point_filter_num for denser clouds
- Verify LiDAR type setting matches your sensor

**Map not saving**
- Check `pcd_save_en: true` in config
- Verify `PCD/` directory exists and is writable
- Use Ctrl+C to trigger save on exit

---

## 📚 Related Works

### SLAM Systems
1. [**ikd-Tree**](https://github.com/hku-mars/ikd-Tree) - Dynamic KD-Tree
2. [**R2LIVE**](https://github.com/hku-mars/r2live) - LiDAR-Inertial-Visual fusion
3. [**LI_Init**](https://github.com/hku-mars/LiDAR_IMU_Init) - LiDAR-IMU calibration
4. [**FAST-LIO-LOCALIZATION**](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) - Re-localization module

### Control and Planning
1. [**IKFoM**](https://github.com/hku-mars/IKFoM) - On-manifold Kalman filter
2. [**UAV Obstacle Avoidance**](https://github.com/hku-mars/dyn_small_obs_avoidance) - Planning demo
3. [**Bubble Planner**](https://arxiv.org/abs/2202.12177) - Quadrotor planning

---

## 📖 Publications

### FAST-LIO2
```bibtex
@article{xu2022fast,
  title={Fast-lio2: Fast direct lidar-inertial odometry},
  author={Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  volume={38},
  number={4},
  pages={2053--2073},
  year={2022}
}
```

### FAST-LIO
```bibtex
@article{xu2021fast,
  title={Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter},
  author={Xu, Wei and Zhang, Fu},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={3317--3324},
  year={2021}
}
```

---

## 👥 Contributors

- [Wei Xu 徐威](https://github.com/XW-HKU)
- [Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)
- [Dongjiao He 贺东娇](https://github.com/Joanna-HE)
- [Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)
- [Jiarong Lin 林家荣](https://github.com/ziv-lin)
- [Zheng Liu 刘政](https://github.com/Zale-Liu)
- [Borong Yuan](https://github.com/borongyuan)
- [Ericsii](https://github.com/Ericsii) (ROS2 port maintainer)

---

## 📝 License

This project is licensed under the GPLv2 License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- HKU Mars Lab for the original FAST-LIO development
- Livox Technology for LiDAR hardware and SDK support
- ROS2 and Open Robotics community

---

**Happy Mapping! 🗺️**
