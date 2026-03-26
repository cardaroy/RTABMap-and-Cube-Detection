# Intel RealSense Setup (ROS 2 Humble on WSL2 / Ubuntu 22.04)

## 0. WSL2: Pass USB Camera to Linux (required every reboot)

WSL does not see USB devices by default. Use `usbipd-win` to forward the RealSense.

### One-time: Install usbipd on Windows

Open **Windows PowerShell (Admin)** and run:

```powershell
winget install usbipd
```

### Every time you plug in the camera

In **Windows PowerShell (Admin)**:

```powershell
# List USB devices — find the RealSense (Intel(R) RealSense)
usbipd list

# Bind it (only needed once per device, use the BUSID from above e.g. 2-4)
usbipd bind --busid <BUSID>

# Attach it to WSL
usbipd attach --wsl --busid <BUSID>
```

Then verify **in WSL**:

```bash
lsusb | grep -i intel
```

You should see something like `Intel Corp. Intel(R) RealSense(TM)...`

> **Note:** If you unplug/replug the camera, you need to run `usbipd attach --wsl --busid <BUSID>` again.

---

## 1. Install Packages (in WSL)

```bash
sudo apt update
sudo apt install ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
```

## 2. Install udev Rules (in WSL)

```bash
sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 3. Verify Camera Detection (in WSL)

```bash
lsusb | grep -i intel
rs-enumerate-devices | head -20
ls /dev/video*
```

## 4. Launch RealSense Node

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

With depth, color, and pointcloud enabled:

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:="" \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  rgb_camera.color_profile:=424x240x30 \
  depth_module.depth_profile:=424x240x30 \
  depth_module.emitter_enabled:=1 \
  enable_sync:=true \
  align_depth.enable:=true \
  decimation_filter.enable:=true \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true \
  hole_filling_filter.enable:=false \
  pointcloud.enable:=false \
  publish_tf:=true \
  clip_distance:=4.0
```

## 5. Verify Topics

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
```

## Troubleshooting

- **No device detected in WSL**: Run `usbipd list` in PowerShell to confirm it's attached. Re-run `usbipd attach --wsl --busid <BUSID>`.
- **No device after reboot**: You must re-run `usbipd attach` after every WSL restart or camera replug.
- **Permission denied**: Make sure udev rules are installed (step 2) and you've re-plugged the camera.
- **Low FPS / USB 2 warning**: Use a USB 3.0 port. Avoid hubs.
- **Grey screen in camera app**: Camera works but needs USB passthrough to WSL — complete step 0.
