import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()
    
    sys_id = int(os.getenv('SYS_ID'))
    
    node = Node(package = 'pixhawk_interface',
                name = 'pixhawk_position_interface',
                executable = 'position',
                parameters = [{'sys_id': sys_id}])

    ld.add_action(node)

    return ld
