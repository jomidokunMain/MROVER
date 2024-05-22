
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='run_neural',
            executable='run_neural',
            output='screen',
            parameters=[{'weight_file_name': 'path/to/weight_file'}]
        )
    ])