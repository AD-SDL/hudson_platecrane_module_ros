
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    vendor_id = LaunchConfiguration('vendor_id')
    product_id = LaunchConfiguration('product_id')

    declare_use_vendor_id_cmd = DeclareLaunchArgument(
        name = 'vendor_id',
        default_value = "146.137.240.35",
        description = 'Flag to accept vendor_id address')

    declare_use_product_id_cmd = DeclareLaunchArgument(
        name = 'product_id',
        default_value = "10100",
        description = 'Flag to accept product_id number')

    scisclops_client = Node(
            package='sciclops_module_client',
            namespace = 'std_ns',
            executable='sciclopsNode',
            name='SciclopsNode',
            parameters=[
                {'vendor_id':vendor_id},
                {'product_id':product_id}
                ],
            emulate_tty=True
    )

    
    launch_d = LaunchDescription()

    launch_d.add_action(declare_use_vendor_id_cmd)
    launch_d.add_action(declare_use_product_id_cmd)
    launch_d.add_action(scisclops_client)
    
    return launch_d
    
