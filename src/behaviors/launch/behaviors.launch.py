import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def loadBehaviors(path):
    with open(path, "r") as f:
        behavior_dict = yaml.safe_load(f)

    return behavior_dict

def generate_launch_description():

    ld = LaunchDescription()

    path = os.path.join(get_package_share_directory('behaviors'),
                          'config','behaviors.yaml')

    behavior_dict = loadBehaviors(path)
    
    behavior_name = input("What behavior do you want to run: ")
    
    file_name = behavior_dict[behavior_name]["filename"]
    class_name = behavior_dict[behavior_name]["class"]
    
    params = behavior_dict[behavior_name]["params"]

    params['sys_id'] = 1
    
    node = Node(package = 'behaviors',
                name = 'behavior_node',
                executable = 'behavior_ros_interface_node.py',
                parameters = [params],
                arguments = [file_name,class_name]
                )

    ld.add_action(node)

    return ld
                
