# map_depth_eval

This package projects a live `PointCloud2` stream into a camera using `CameraInfo` plus TF, publishes the synthetic depth image, and backprojects that synthetic depth image into a cloud for RViz comparison.

It also includes a `tabletop_depth_enhancer` node for post-processing aligned RealSense depth in a narrow tabletop range.

## Confirmed from your bag

- `/camera/camera/color/camera_info` uses `camera_color_optical_frame`
- `/tf_static` contains:
  - `camera_link -> camera_color_frame`
  - `camera_color_frame -> camera_color_optical_frame`
  - `camera_link -> camera_depth_frame`
  - `camera_depth_frame -> camera_depth_optical_frame`
- The recorded bag does **not** contain a transform from `body` or `base_link` to `camera_link`

Without `body/base_link -> camera_link`, the projection cannot be geometrically correct.

## Topics

- Input point cloud: `/livox/lidar/pcd2`
- Input camera info: `/camera/camera/color/camera_info`
- Input color image: `/camera/camera/color/image_raw/compressed`
- Output depth image: `/map_depth/projected_depth`
- Output overlay image: `/map_depth/projected_overlay`
- Output backprojected cloud: `/map_depth/backprojected_cloud`
- Output visible map subset: `/map_depth/visible_map_points`

## Build

```bash
cd /home/rey/unitree_g1_control/ros2_ws
colcon build --packages-select map_depth_eval
source install/setup.bash
```

## Run with a live point cloud

Terminal 1:

```bash
cd /home/rey/unitree_g1_control/ros2_ws
source install/setup.bash
ros2 launch map_depth_eval map_depth_eval.launch.py \
  pointcloud_topic:=/your/pointcloud/topic \
  camera_info_topic:=/camera/camera/color/camera_info \
  image_topic:=/camera/camera/color/image_raw/compressed
```

If your live TF tree does not already publish `body2 -> camera_link`, enable the built-in static TF publisher:

```bash
ros2 launch map_depth_eval map_depth_eval.launch.py \
  pointcloud_topic:=/your/pointcloud/topic \
  publish_camera_mount_tf:=true \
  camera_parent_frame:=body2 \
  camera_link_frame:=camera_link \
  camera_tx:=0.056744402376091595 \
  camera_ty:=0.0175 \
  camera_tz:=0.01598012825293377 \
  camera_roll:=0.0 \
  camera_pitch:=0.7906341511534315 \
  camera_yaw:=0.0
```

Those values come from the URDF mounts:

- `torso_link -> d435_link`: `xyz=(0.0576235, 0.01753, 0.41987)`, `rpy=(0, 0.8307767239493009, 0)`
- `torso_link -> mid360_link`: `xyz=(0.0002835, 0.00003, 0.40618)`, `rpy=(0, 0.04014257279586953, 0)`

and the derived relative transform:

- `mid360_link -> d435_link`: `xyz=(0.056744402376091595, 0.0175, 0.01598012825293377)`, `rpy=(0, 0.7906341511534315, 0)`

Terminal 2:

```bash
cd /home/rey/unitree_g1_control/ros2_ws
source install/setup.bash
ros2 bag play rosbag2_2026_03_25-13_17_34 --clock
```

## RViz

Use fixed frame `map` and add:

- `Image` on `/map_depth/projected_depth`
- `Image` on `/map_depth/projected_overlay`
- `PointCloud2` on your input cloud topic
- `PointCloud2` on `/map_depth/backprojected_cloud`
- `PointCloud2` on `/map_depth/visible_map_points`

The closer `/map_depth/backprojected_cloud` is to the visible subset of the original input cloud, the more consistent the cloud, intrinsics, and camera extrinsics are.

## Tabletop Depth Enhancement

Run:

```bash
cd /home/rey/unitree_g1_control/ros2_ws
source install/setup.bash
ros2 launch map_depth_eval tabletop_depth_enhancer.launch.py
```

Defaults:

- input depth: `/camera/camera/aligned_depth_to_color/image_raw`
- filtered depth: `/tabletop_depth/filtered`
- enhanced visualization: `/tabletop_depth/enhanced_viz`
- in-range mask: `/tabletop_depth/range_mask`
- range: `0.70 m` to `1.00 m`

Example for a tighter tabletop band:

```bash
ros2 launch map_depth_eval tabletop_depth_enhancer.launch.py \
  d_min:=0.78 \
  d_max:=0.92
```
