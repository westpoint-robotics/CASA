import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()
    
    sys_id = int(os.getenv('ROS_DOMAIN_ID'))

    micro_ros_node = Node(package = "micro_ros_agent",
                          name = "micro_ros",
                          executable = "micro_ros_agent",
                          arguments = ['--dev', '/dev/ttyUSB0', '-b', '921600'])
    
    node = Node(package = 'pixhawk_interface',
                name = 'pixhawk_position_interface',
                executable = 'position',
                parameters = [{'sys_id': sys_id}])

    sys_node =  Node(package = 'system_interface',
                     name = 'system_interface',
                     executable = 'system',
                     parameters = [{'sys_id': sys_id}])
    
    ld.add_action(node)
    ld.add_action(sys_node)

    return ld
