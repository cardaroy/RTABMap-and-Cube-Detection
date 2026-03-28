"""
Combined launch: RTAB-Map SLAM + Cube Detection + Auto-Save.

Camera runs separately on the Jetson:
  ros2 launch realsense2_camera rs_launch.py ...

Then on the workstation:
  ros2 launch src/full_slam_autosave_launch.py rtabmap_viz:=true

Options:
  rviz:=true              Open RViz2
  rtabmap_viz:=true       Open RTAB-Map UI
  cube_detection:=true    Enable cube detector + map markers (default true)
  show_debug:=true        Show OpenCV debug window for detections (default true)
  localization:=false     Localization-only mode (no new mapping)
  save_interval:=30       Auto-save interval in seconds (default 30)
  save_dir:=~/my_new_ws/saves   Output directory for auto-saved files
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([

        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Launch RTAB-Map UI'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization mode (no new mapping)'),
        DeclareLaunchArgument('cube_detection', default_value='true',
                              description='Launch cube detector + map marker nodes'),
        DeclareLaunchArgument('show_debug', default_value='true',
                              description='Show OpenCV debug window for cube detections'),
        DeclareLaunchArgument('save_interval', default_value='30',
                              description='Auto-save interval in seconds'),
        DeclareLaunchArgument('save_dir',
                              default_value=os.path.expanduser('~/my_new_ws/saves'),
                              description='Output directory for auto-saved files'),

        # ── RTAB-Map SLAM + odometry + viz ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_THIS_DIR, 'rtabmap_optimized_launch.py')
            ),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'localization': LaunchConfiguration('localization'),
            }.items(),
        ),

        # ── Cube detection (optional) ─────────────────────────────
        Node(
            package='cube_detection',
            executable='cube_detector_standard_node',
            output='screen',
            parameters=[{
                'model_path': '/home/cardaroy/my_new_ws/Yao_Node_2/yolomodel_python/best.pt',
                'show_debug': LaunchConfiguration('show_debug'),
            }],
            condition=IfCondition(LaunchConfiguration('cube_detection')),
        ),

        Node(
            package='cube_detection',
            executable='cube_map_marker_node',
            output='screen',
            parameters=[{
                'min_observations': 10,
            }],
            condition=IfCondition(LaunchConfiguration('cube_detection')),
        ),

        # ── Auto-save (cloud + markers) ───────────────────────────
        ExecuteProcess(
            cmd=[
                'python3', os.path.join(_THIS_DIR, 'auto_save.py'),
                '--interval', LaunchConfiguration('save_interval'),
                '--dir', LaunchConfiguration('save_dir'),
            ],
            output='screen',
        ),
    ])
