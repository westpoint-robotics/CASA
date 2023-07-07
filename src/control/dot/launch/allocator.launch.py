import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('dot'),
                          'config','task.yaml')
    
    for arg in sys.argv:
        if arg.startswith("num_agents:="):
            num_agents = int(arg.split(':=')[1])

    for i in range(1,num_agents+1):
        node = Node(package = 'dot',
                    name = 'casa_'+str(i)+'_allocator',
                    executable = 'allocator_node.py',
                    parameters = [{'sys_id': i,
                                   'threshold': 3.0,
                                   'path': 'casa_ws/src/control/dot/tasks/'}])
        
        ld.add_action(node)

    return ld
