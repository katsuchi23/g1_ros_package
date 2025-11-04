# FAST_LIO Automatic PCD Save Fix

## Issue Fixed
The PCD file was not being saved automatically after mapping completion, even with `pcd_save_en: true` in the configuration.

## Root Cause
The shutdown handler was trying to save from `pcl_wait_save` pointer, which was never populated because the code to accumulate points into it was commented out. Meanwhile, `pcl_wait_pub` was being actively populated during the mapping process.

## Solution Applied
Changed the shutdown handler in `laserMapping.cpp` (line ~1170) to save from `pcl_wait_pub` instead of `pcl_wait_save`.

## How It Works Now
1. During mapping, point clouds are accumulated in `pcl_wait_pub` (via the `publish_map()` function)
2. When you stop the node (Ctrl+C), the shutdown handler automatically saves the accumulated map
3. The PCD file is saved to: `<FAST_LIO_directory>/PCD/scans.pcd`

## Configuration
Make sure these settings are in your config file (`config/mid360.yaml`):

```yaml
pcd_save:
    pcd_save_en: true      # Enable automatic saving
    interval: -1           # -1 = save all frames in one file
```

## Usage
1. Launch FAST_LIO mapping:
   ```bash
   ros2 launch fast_lio mapping.launch.py
   ```

2. Drive around and create your map

3. When finished, press **Ctrl+C** to stop the node

4. The map will automatically be saved to:
   ```
   <ros2_ws>/src/mapping/FAST_LIO/PCD/scans.pcd
   ```

5. You should see this message in the terminal:
   ```
   current scan saved to /PCD/scans.pcd
   ```

## Alternative: Save via Service Call
You can also manually save the map while the node is running:

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

This will save the map to the path specified in `map_file_path` configuration parameter.

## Notes
- The file will be saved in binary PCD format
- Make sure you have enough memory for large maps
- The `interval` parameter can be set to a positive number (e.g., 100) to save multiple smaller PCD files instead of one large file
