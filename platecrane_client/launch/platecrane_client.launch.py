
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    port = LaunchConfiguration('port')

    declare_use_port_cmd = DeclareLaunchArgument(
        name = 'port',
        default_value = "/dev/ttyUSB2",
        description = 'Flag to accept port number')

    platecrane_client = Node(
            package='platecrane_client',
            namespace = 'std_ns',
            executable='platecrane_client',
            name='PlatecraneNode',
            parameters=[
                {'port':port},
                ],
            emulate_tty=True
    )

    
    launch_d = LaunchDescription()

    launch_d.add_action(declare_use_port_cmd)
    launch_d.add_action(platecrane_client)
    
    return launch_d
    
