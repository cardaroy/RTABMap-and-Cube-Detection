[[NEEDS TESTING]]


ssh rover@192.168.1.161 ??? password 1234
source /home/rover/ros2_humble/install/setup.bash ???? I think??
source /home/rover/rover_ws/install/setup.bash ???? I think??

then pray that the jetson can run realsense
not sure what directory to run this from


ros2 launch realsense2_camera rs_launch.py \
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

  then pray to god that the topics are outputted