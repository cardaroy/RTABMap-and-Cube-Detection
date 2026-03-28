"""
Optimized RealSense D435i launch for RTAB-Map SLAM on WSL2.

Tunings:
  - Lower resolution (424x240) for depth to reduce CPU/bandwidth on WSL
  - Aligned depth enabled for RGBD
  - Decimation + spatial + temporal filters for cleaner depth
  - IR emitter on for better depth in low-texture scenes
  - IMU enabled + fused with madgwick filter
  - Pointcloud disabled here (let RTAB-Map generate it)
  - Sync enabled for matched RGB + depth frames

Usage:
  ros2 launch realsense_optimized_launch.py
  ros2 launch realsense_optimized_launch.py rgb_resolution:=640x360x30
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument('rgb_resolution', default_value='424x240x30',
                              description='RGB profile: WxHxFPS'),
        DeclareLaunchArgument('depth_resolution', default_value='424x240x30',
                              description='Depth profile: WxHxFPS'),
        DeclareLaunchArgument('unite_imu_method', default_value='2',
                              description='0=None, 1=copy, 2=linear_interpolation'),

        # IR emitter on for better depth
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # --- RealSense camera driver ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'),
                             'launch', 'rs_launch.py')
            ]),
            launch_arguments={
                # Streams
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                # Resolution / FPS
                'rgb_camera.color_profile': LaunchConfiguration('rgb_resolution'),
                'depth_module.depth_profile': LaunchConfiguration('depth_resolution'),
                # Sync & alignment
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                # IMU
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                # Filters — clean up noisy depth
                'decimation_filter.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'hole_filling_filter.enable': 'false',
                # No pointcloud from driver (RTAB-Map does this)
                'pointcloud.enable': 'false',
                # TF
                'publish_tf': 'true',
                # Clip far depth (meters, -2 = disabled)
                'clip_distance': '4.0',
            }.items(),
        ),

        # --- IMU filter (Madgwick) ---
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[('imu/data_raw', '/camera/camera/imu')],
        ),
    ])
