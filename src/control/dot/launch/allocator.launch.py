import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()


    config = os.path.join(get_package_share_directory('dot'),
                          'config','task.yaml')
    sys_id = 1
    
    node1 = Node(package = 'dot',
                name = 'allocator',
                executable = 'allocator_node.py',
                parameters = [{'sys_id': sys_id}, config])
    
    ld.add_action(node1)

    return ld
