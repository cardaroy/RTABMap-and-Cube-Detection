"""
Optimized RTAB-Map SLAM launch for RealSense on WSL2.

Lighter than defaults:
  - Lower map update rate
  - Reduced memory usage (fewer nodes kept)
  - Grid map only (no 3D cloud map publishing)
  - Faster loop closure detection
  - Paired with realsense_optimized_launch.py for camera

Usage:
  Terminal 1: ros2 launch src/realsense_optimized_launch.py
  Terminal 2: ros2 launch src/rtabmap_optimized_launch.py
  Or with rviz: ros2 launch src/rtabmap_optimized_launch.py rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # RTAB-Map parameters tuned for performance
    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        # 'wait_imu_to_init': True,  # uncomment for D435i with IMU

        # --- Memory / speed ---
        'Mem/STMSize': '10',              # Short-term memory size (default 10, keep low)
        'Mem/IncrementalMemory': 'true',

        # --- Loop closure ---
        'Rtabmap/DetectionRate': '1.0',   # Hz — detect loop closures at 1Hz instead of every frame
        'Rtabmap/TimeThr': '700',         # ms — max time for detection (drop frames if slower)

        # --- Visual features ---
        'Vis/MaxFeatures': '600',         # default 1000 — balance between speed and stability
        'Vis/MinInliers': '15',           # min feature matches for valid transform

        # --- RGBD odometry (stable) ---
        'Odom/Strategy': '0',             # 0=Frame-to-Map (more robust)
        'OdomF2M/MaxSize': '2000',        # max features in local map (default 3000)
        'Odom/GuessMotion': 'true',       # use previous motion as guess
        'Odom/KeyFrameThr': '0.4',        # new keyframe when 40% features change (smoother)
        'RGBD/OptimizeMaxError': '3.0',   # reject transforms with error > 3m (prevents jumps)
        'Odom/ResetCountdown': '2',       # auto-reset odom after 2 lost frames instead of freezing

        # --- Grid map (2D) ---
        'Grid/FromDepth': 'true',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/RangeMax': '4.0',           # match camera clip distance
        'Grid/CellSize': '0.1',           # 10cm cells (coarser = faster, default 0.05)

        # --- 3D cloud map ---
        'Grid/3D': 'true',

        # --- Reduce database size ---
        'Mem/ImagePreDecimation': '1',    # no decimation (full resolution clouds)
        'Mem/ImagePostDecimation': '2',   # decimate images before saving
    }]

    remappings = [
        # ('imu', '/imu/data'),  # uncomment for D435i with IMU
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
    ]

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch rviz2'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Launch RTAB-Map UI'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization mode (no new mapping)'),

        # --- Visual odometry ---
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings,
        ),

        # --- SLAM ---
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=rtabmap_parameters + [{
                'cloud_map_publish_rate': 1.0,   # Publish /rtabmap/cloud_map at 1 Hz
                'grid_map_publish_rate': 1.0,     # Publish /rtabmap/grid_map at 1 Hz
            }],
            remappings=remappings,
            arguments=['-d'],  # delete previous DB on start (remove for resume)
        ),

        # --- Visualization (optional) ---
        
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', '/home/cardaroy/my_new_ws/src/rtabmap_rviz.rviz'],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
    
