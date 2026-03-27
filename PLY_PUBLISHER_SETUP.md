

then in another terminal run this:

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map ply_origin

then publish map

python3 src/ply_publisher.py /home/cardaroy/cloudvisuals/cloud3.ply 

replace /home/cardaroy/cloudvisuals/cloud3.ply with actual file path

then 
python3 src/ply_publisher.py /home/cardaroy/cloudvisuals/cloud3.ply 

python3 src/load_cube_markers.py markers.json