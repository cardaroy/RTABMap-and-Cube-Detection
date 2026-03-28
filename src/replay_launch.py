"""
Presentation replay: load saved PLY cloud + cube markers in RViz.

Usage:
  ros2 launch src/replay_launch.py
  ros2 launch src/replay_launch.py ply:=saves/cloud_20260328_120000.ply markers:=saves/markers_20260328_120000.json

Options:
  ply:=saves/cloud_latest.ply           Path to PLY file
  markers:=saves/markers_latest.json    Path to markers JSON
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.dirname(_THIS_DIR)


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('ply',
                              default_value=os.path.join(_WS, 'saves', 'cloud_latest.ply'),
                              description='Path to PLY point cloud file'),
        DeclareLaunchArgument('markers',
                              default_value=os.path.join(_WS, 'saves', 'markers_latest.json'),
                              description='Path to cube markers JSON file'),

        # Static TF so RViz accepts the 'map' frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'ply_origin'],
            output='screen',
        ),

        # PLY point cloud publisher
        ExecuteProcess(
            cmd=['python3', os.path.join(_THIS_DIR, 'ply_publisher.py'),
                 LaunchConfiguration('ply')],
            output='screen',
        ),

        # Cube markers publisher
        ExecuteProcess(
            cmd=['python3', os.path.join(_THIS_DIR, 'load_cube_markers.py'),
                 LaunchConfiguration('markers')],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(_THIS_DIR, 'rtabmap_rviz.rviz')],
        ),
    ])
