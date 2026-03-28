source install/setup.bash

# Simple node (bbox + depth only):
ros2 run cube_detection cube_detector_standard_node --ros-args \
  -p model_path:=/home/cardaroy/my_new_ws/Yao_Node_2/yolomodel_python/best.pt

# Standard node (full 3D pose):
ros2 run cube_detection cube_detector_standard_node --ros-args -p model_path:=/home/cardaroy/my_new_ws/Yao_Node_2/yolomodel_python/best.pt

# Or via launch file (simple node by default):
ros2 launch cube_detection cube_detection.launch.py

# Persistent map markers (run alongside standard node + RTAB-Map):
# Transforms detections into map frame so cubes stay fixed on the 3D map
ros2 run cube_detection cube_map_marker_node
# Then in RViz: Add → By topic → /perception/cube_map_markers → MarkerArray



source install/setup.bash
ros2 run cube_detection cube_detector_standard_node --ros-args \
  -p model_path:=/home/cardaroy/my_new_ws/Yao_Node_2/yolomodel_python/best.pt
