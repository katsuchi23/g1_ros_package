# FAST-LIO-LOCALIZATION (ROS2)# FAST-LIO-LOCALIZATION



> **ROS2 Migration**: This is a ROS2 Humble port of the original [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)A simple localization framework that can re-localize in built maps based on [FAST-LIO](https://github.com/hku-mars/FAST_LIO). 



A robust localization framework that enables re-localization in pre-built point cloud maps using [FAST-LIO](https://github.com/hku-mars/FAST_LIO).## News



[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)- **2021-08-11:** Add **Open3D 0.7** support.

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)  

[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)- **2021-08-09:** Migrate to **Open3D** for better performance.



<div align="center">## 1. Features

<img src="doc/demo.GIF" width=49% />- Realtime 3D global localization in a pre-built point cloud map. 

<img src="doc/demo_accu.GIF" width=49% />  By fusing low-frequency global localization (about 0.5~0.2Hz), and high-frequency odometry from FAST-LIO, the entire system is computationally efficient.

</div>

<div align="center"><img src="doc/demo.GIF" width=90% /></div>

## ✨ Features

- Eliminate the accumulative error of the odometry.

- **Real-time Global Localization**: Re-localize in pre-built point cloud maps at ~0.2-0.5Hz

- **High-Frequency Odometry**: Fuses global localization with FAST-LIO odometry for smooth tracking<div align="center"><img src="doc/demo_accu.GIF" width=90% /></div>

- **Eliminates Drift**: Corrects accumulative odometry errors through map matching

- **Flexible Initialization**: Manual initialization via RViz or automatic with sensor fusion- The initial localization can be provided either by rough manual estimation from RVIZ, or pose from another sensor/algorithm.

- **Multi-LiDAR Support**: Livox (Avia, Horizon, Mid-360), Velodyne, Ouster

<!-- ![image](doc/real_experiment2.gif) -->

<div align="center"><!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->

<img src="doc/demo_init.GIF" width=49% /><div align="center">

<img src="doc/demo_init_2.GIF" width=49% /><img src="doc/demo_init.GIF" width=49.6% />

</div><img src="doc/demo_init_2.GIF" width = 49.6% >

</div>

---



## 📋 Table of Contents## 2. Prerequisites

### 2.1 Dependencies for FAST-LIO

- [Prerequisites](#prerequisites)

- [Installation](#installation)Technically, if you have built and run FAST-LIO before, you may skip section 2.1.

- [Quick Start](#quick-start)

- [Configuration](#configuration)This part of dependency is consistent with FAST-LIO, please refer to the documentation https://github.com/hku-mars/FAST_LIO#1-prerequisites

- [Advanced Usage](#advanced-usage)

- [Troubleshooting](#troubleshooting)### 2.2 Dependencies for localization module

- [Related Works](#related-works)

- python 2.7

---

- [ros_numpy](https://github.com/eric-wieser/ros_numpy)

## 🔧 Prerequisites

```shell

### System Requirementssudo apt install ros-$ROS_DISTRO-ros-numpy

- **OS**: Ubuntu 22.04 (or 20.04)```

- **ROS2**: Humble (Foxy/Galactic also supported)

- **Memory**: >= 8GB RAM recommended- [Open3D](http://www.open3d.org/docs/0.9.0/getting_started.html)

- **CPU**: Multi-core processor recommended

```shell

### Dependenciespip install open3d==0.9

```

#### ROS2 Packages

```bashNotice that, there may be issue when installing **Open3D** directly using pip in **Python2.7**: 

sudo apt install -y \```shell

    ros-humble-desktop \ERROR: Package 'pyrsistent' requires a different Python: 2.7.18 not in '>=3.5'

    ros-humble-pcl-ros \```

    ros-humble-pcl-conversions \you may firstly install **pyrsistent**:

    ros-humble-tf2-eigen \```shell

    ros-humble-nav2-map-serverpip install pyrsistent==0.15

``````

Then

#### Python Dependencies```shell

```bashpip install open3d==0.9

# Python 3.8+```

pip3 install numpy scipy open3d

```

## 3. Build

#### PCL and EigenClone the repository and catkin_make:

```bash

sudo apt install -y \```

    libpcl-dev \    cd ~/$A_ROS_DIR$/src

    libeigen3-dev    git clone https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION.git

```    cd FAST_LIO_LOCALIZATION

    git submodule update --init

---    cd ../..

    catkin_make

## 📥 Installation    source devel/setup.bash

```

### 1. Create Workspace- Remember to source the livox_ros_driver before build (follow [livox_ros_driver](https://github.com/hku-mars/FAST_LIO#13-livox_ros_driver))

- If you want to use a custom build of PCL, add the following line to ~/.bashrc

```bash  ```export PCL_ROOT={CUSTOM_PCL_PATH}```

mkdir -p ~/lidar_ws/src

cd ~/lidar_ws/src

```## 4. Run Localization

### 4.1 Sample Dataset

### 2. Clone Repository

Demo rosbag in a large underground garage: 

```bash[Google Drive](https://drive.google.com/file/d/15ZZAcz84mDxaWviwFPuALpkoeK-KAh-4/view?usp=sharing) | [Baidu Pan (Code: ne8d)](https://pan.baidu.com/s/1ceBiIAUqHa1vY3QjWpxwNA);

git clone --recursive https://github.com/YOUR_USERNAME/FAST_LIO_LOCALIZATION.git

cd FAST_LIO_LOCALIZATIONCorresponding map: [Google Drive](https://drive.google.com/file/d/1X_mhPlSCNj-1erp_DStCQZfkY7l4w7j8/view?usp=sharing) | [Baidu Pan (Code: kw6f)](https://pan.baidu.com/s/1Yw4vY3kEK8x2g-AsBi6VCw)

git submodule update --init --recursive

```The map can be built using LIO-SAM or FAST-LIO-SLAM.



### 3. Install Dependencies### 4.2 Run



```bash1. First, please make sure you're using the **Python 2.7** environment;

cd ~/lidar_ws

rosdep install --from-paths src --ignore-src -r -y

```2. Run localization, here we take Livox AVIA as an example:



### 4. Build```shell

roslaunch fast_lio_localization localization_avia.launch map:=/path/to/your/map.pcd

```bash```

cd ~/lidar_ws

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=ReleasePlease modify `/path/to/your/map.pcd` to your own map point cloud file path.

source install/setup.bash

```Wait for 3~5 seconds until the map cloud shows up in RVIZ;



**Note**: Make sure `livox_ros_driver2` is built and sourced before building this package.3. If you are testing with the sample rosbag data:

```shell

---rosbag play localization_test_scene_1.bag

```

## 🚀 Quick Start

Or if you are running realtime

### 1. Prepare Your Map

```shell

You need a PCD map file. You can create one using:roslaunch livox_ros_driver livox_lidar_msg.launch

- [FAST-LIO](../FAST_LIO)```

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)Please set the **publish_freq** in **livox_lidar_rviz.launch** to **10Hz**, to ensure there are enough points for global localization in a single scan. 

- [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)Support for higher frequency is coming soon.



Place your map file in the `PCD/` directory or specify the path when launching.4. Provide initial pose

```shell

### 2. Launch Localizationrosrun fast_lio_localization publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0 

```

#### For Livox Mid-360:The numerical value **14.5 -7.5 0 -0.25 0 0** denotes 6D pose **x y z yaw pitch roll** in map frame, 

```bashwhich is a rough initial guess for **localization_test_scene_1.bag**. 

ros2 launch fast_lio_localization localization_mid360.launch.py \

    map:=/path/to/your/map.pcdThe initial guess can also be provided by the '2D Pose Estimate' Tool in RVIZ.

```

Note that, during the initialization stage, it's better to keep the robot still. Or if you play bags, fistly play the bag for about 0.5s, and then pause the bag until the initialization succeed. 

#### For Livox Avia:

```bash

ros2 launch fast_lio_localization localization_avia.launch.py \## Related Works

    map:=/path/to/your/map.pcd1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package

```2. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.

3. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): The integration of FAST-LIO with [Scan-Context](https://github.com/irapkaist/scancontext) **loop closure** module.

#### For Velodyne:4. [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization): A simple system that can relocalize a robot on a built map based on LIO-SAM.

```bash

ros2 launch fast_lio_localization localization_velodyne.launch.py \

    map:=/path/to/your/map.pcd## Acknowledgments

```Thanks for the authors of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization).



### 3. Set Initial Pose## TODO

1. Go over the timestamp issue of the published odometry and tf;

Wait 3-5 seconds for the map to load in RViz, then set the initial pose:2. Using integrated points for global localization;

3. Fuse global localization with the state estimation of FAST-LIO, and smooth the localization trajectory; 

#### Method 1: Using RViz4. Updating...
- Click "2D Pose Estimate" button in RViz toolbar
- Click and drag on the map to set position and orientation

#### Method 2: Using Command Line
```bash
ros2 run fast_lio_localization publish_initial_pose.py <x> <y> <z> <yaw> <pitch> <roll>

# Example
ros2 run fast_lio_localization publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0
```

### 4. Start Data Input

#### With Live LiDAR:
```bash
# Livox LiDAR
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

#### With Rosbag (for testing):
```bash
# Use the launch file's built-in rosbag player
ros2 launch fast_lio_localization localization_mid360.launch.py \
    map:=/path/to/map.pcd \
    use_bag:=true \
    bag_path:=/path/to/rosbag_folder

# Or manually
ros2 bag play your_bag.db3 --clock
```

---

## ⚙️ Configuration

### Launch File Parameters

All launch files support these parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map` | `PCD/mapping.pcd` | Path to PCD map file |
| `rviz` | `true` | Launch RViz visualization |
| `use_bag` | `false` | Enable rosbag playback |
| `bag_path` | (auto) | Path to rosbag folder |
| `bag_rate` | `1.0` | Playback rate multiplier |

### Config Files

Configuration files are in `config/` directory:

- **mid360.yaml** - Livox Mid-360 settings
- **avia.yaml** - Livox Avia settings
- **horizon.yaml** - Livox Horizon settings
- **velodyne.yaml** - Velodyne LiDAR settings
- **ouster64.yaml** - Ouster LiDAR settings

### Key Parameters

```yaml
/**:
  ros__parameters:
    # Point cloud filtering
    point_filter_num: 3           # Downsample factor (higher = faster, lower accuracy)
    filter_size_surf: 0.5         # Surface point filter size
    filter_size_map: 0.5          # Map point filter size
    
    # LiDAR topics
    common:
      lid_topic: "/livox/lidar"   # LiDAR point cloud topic
      imu_topic: "/livox/imu"     # IMU topic
      
    # Mapping parameters
    mapping:
      det_range: 100.0            # Detection range (meters)
      fov_degree: 360.0           # Field of view (degrees)
      
    # Global localization
    # Uses ICP scan matching at ~0.2-0.5 Hz
```

---

## 📊 Topics and TF

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR point cloud |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU measurements |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/Odometry` | `nav_msgs/msg/Odometry` | Current pose estimate |
| `/path` | `nav_msgs/msg/Path` | Robot trajectory |
| `/map` | `sensor_msgs/msg/PointCloud2` | Loaded PCD map |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered scan |
| `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | Effective points |

### TF Frames
```
map
 └─ odom
     └─ base_link
         └─ lidar_link
```

---

## 🐛 Troubleshooting

See [LOCALIZATION_USAGE.md](LOCALIZATION_USAGE.md) for detailed troubleshooting guide.

### Quick Fixes

**Map Not Loading**: Check file path and permissions
**Poor Accuracy**: Verify initial pose and increase point_filter_num
**High CPU**: Reduce point_filter_num and det_range
**No IMU Data**: Check topic names in config file

---

## 📚 Related Works

1. [**FAST-LIO**](https://github.com/hku-mars/FAST_LIO) - Fast LiDAR-Inertial Odometry
2. [**ikd-Tree**](https://github.com/hku-mars/ikd-Tree) - Dynamic KD-Tree
3. [**FAST-LIO-SLAM**](https://github.com/gisbi-kim/FAST_LIO_SLAM) - With loop closure

---

## 🙏 Acknowledgments

- Original [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) by HViktorTsoi
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) by HKU Mars Lab
- ROS2 migration contributors

---

## 📝 License

MIT License - see [LICENSE](LICENSE) file for details.

---

**Happy Localizing! 🎯**
