
Run along the SLAM session in a separate terminal: 
python3 src/auto_save.py --interval 30 


To replay any save in RViz later
python3 src/ply_publisher.py saves/cloud_latest.ply
python3 src/load_cube_markers.py saves/markers_latest.json