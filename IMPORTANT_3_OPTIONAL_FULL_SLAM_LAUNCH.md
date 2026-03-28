source install/setup.bash
ros2 launch src/full_slam_launch.py rviz:=true rtabmap_viz:=true show_debug:=false\

WITH AUTOSAVE:
ros2 launch src/full_slam_autosave_launch.py rviz:=true rtabmap_viz:=true save_interval:=30