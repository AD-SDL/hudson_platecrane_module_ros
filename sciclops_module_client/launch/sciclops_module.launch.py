from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sciclops_module_client',
            namespace='sciclops_module',
            executable='sciclopsNode',
            name='sciclopsNode'
        ),
    ])