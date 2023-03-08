import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    sys_id = int(os.getenv('SYS_ID'))
    
    node1 = Node(package = 'system_interface',
                name = 'system_interface',
                executable = 'system',
                parameters = [{'sys_id': sys_id}])

    ld.add_action(node1)

    return ld
