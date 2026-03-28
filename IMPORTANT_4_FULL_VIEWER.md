source install/setup.bash
ros2 launch src/replay_launch.py ply:=saves/cloud_latest.ply markers:=saves/markers_latest.json



IF THIS DOESNT WORK THEN RUN THESE 4 
launches:

t1:

rviz2

t2:

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map ply_origin

t3:

python3 src/ply_publisher.py saves/cloud_latest.ply

t4:

python3 src/load_cube_markers.py saves/markers_latest.json