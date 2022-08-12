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

        Node(
            package='sciclops_module_client',
            namespace='sciclops_module',
            executable='masterNode',
            name='masterNode'
        ),

        # Node(
        #     package='sp_module_client',
        #     namespace='sp_module',
        #     executable='cameraNode',
        #     name='cameraNode'
        # ),

    ])