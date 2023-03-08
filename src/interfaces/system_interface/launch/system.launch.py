import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    #sys_id = DeclareLaunchArgument('sys_id', default_value='10')
    
    #config = os.path.join(get_package_share_directory('system_interface'),
    #                      'config',
    #                      'system_params.yaml')
    #print(sys_id)

    #id_num = int(os.getenv('SYS_ID'))
    
    node1 = Node(package = 'system_interface',
                name = 'system_interface',
                executable = 'system',
                parameters = [{'sys_id': 1}])

    ld.add_action(node1)

    return ld
