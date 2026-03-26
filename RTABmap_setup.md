# RTAB-Map Setup (ROS 2 Humble + RealSense on WSL2)

## 1. Install Packages

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros ros-humble-imu-filter-madgwick
```

> `ros-humble-rtabmap-ros` is already installed. `ros-humble-imu-filter-madgwick` is needed for IMU fusion with D435i.

## 2. Launch RTAB-Map with RealSense

Make sure the RealSense camera is attached to WSL first (see REALSENSE_SETUP.md step 0).

### Option A: Pre-built D435i launch (recommended for D435i)

This launches the RealSense driver, RGBD odometry, RTAB-Map SLAM, and visualization all at once:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rtabmap_examples realsense_d435i_color.launch.py
```

### Option B: Generic RTAB-Map + separate RealSense driver

Terminal 1 — Start RealSense:
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.profile:=640x360x30
```

Terminal 2 — Start RTAB-Map:
```bash
source /opt/ros/humble/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rviz:=true \
  rtabmap_viz:=true \
  frame_id:=camera_link \
  subscribe_depth:=true \
  approx_sync:=false \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```
```
ros2 launch src/rtabmap_optimized_launch.py \
  rviz:=true \
  rtabmap_viz:=true \
  frame_id:=camera_link \
  subscribe_depth:=true \
  approx_sync:=false \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info 
```
### Option C: View in rviz2 (separate window)

```bash
rviz2
# Add displays: Image, PointCloud2, Map, TF
# Topics: /rtabmap/cloud_map, /rtabmap/grid_map, /camera/camera/color/image_raw
```

## 3. Useful RTAB-Map Topics

```bash
ros2 topic list | grep rtabmap
```

Key topics:
- `/rtabmap/cloud_map` — 3D point cloud map
- `/rtabmap/grid_map` — 2D occupancy grid
- `/rtabmap/odom` — visual odometry
- `/rtabmap/mapData` — full map data

## 4. Save / Load Maps

Maps are saved to `~/.ros/rtabmap.db` by default.

```bash
# View saved map database
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Launch with existing map (no -d flag = resume mapping)
ros2 launch rtabmap_examples realsense_d435i_color.launch.py

# Launch fresh (delete old map with -d flag, set in launch file by default)
```

## 5. Other RealSense Launch Variants

```bash
# D400 series generic
ros2 launch rtabmap_examples realsense_d400.launch.py

# D435i using infrared stereo (no color)
ros2 launch rtabmap_examples realsense_d435i_infra.launch.py

# D435i stereo
ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py
```

## Troubleshooting

- **"No device detected"**: Camera not attached to WSL. Run `usbipd attach --wsl --busid <BUSID>` in Windows PowerShell.
- **"imu_filter_madgwick_node not found"**: Run `sudo apt install ros-humble-imu-filter-madgwick`.
- **Drift / poor odometry**: Ensure good lighting, textured environment, and `align_depth` is enabled.
- **RTAB-Map GUI doesn't open**: WSLg issue — try `export LIBGL_ALWAYS_SOFTWARE=1` before launching.