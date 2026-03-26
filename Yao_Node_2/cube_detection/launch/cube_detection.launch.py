from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Simple perception node
        Node(
            package='cube_detection',
            executable='cube_detector_simple_node',
            name='cube_detector_simple_node',
            output='screen',
            parameters=[{
                'model_path': '/home/yao/models/best.pt',
                'conf_thres': 0.6,
            }]
        ),

        # ---- OR ----
        # If you want standard version instead,
        # comment above and uncomment below:

        # Node(
        #     package='cube_detection',
        #     executable='cube_detector_standard_node',
        #     name='cube_detector_standard_node',
        #     output='screen',
        #     parameters=[{
        #         'model_path': '/home/yao/models/best.pt',
        #         'conf_thres': 0.6,
        #     }]
        # ),

    ])