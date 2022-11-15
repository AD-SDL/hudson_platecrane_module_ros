from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sciclops_module_client',
            namespace = 'std_ns',
            executable='sciclopsNode',
            name='sciclopsNode'
        ),
    ])
