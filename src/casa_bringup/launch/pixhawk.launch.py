import os
import sys

from casa_bringup.network import scan

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    names = ("ubuntu.net")
    network_interface = "wlo1"

    for arg in sys.argv:
        if arg.startswith("name:="):
            names = tuple(arg.replace(' ','').split(','))
        elif arg.startswith("network:="):
            network_interface = str(arg.split(":=")[1])

    ids = scan(names, network_interface)
    sys_id = myIP(network_interface).split('.')[-1]
    
    os.environ["ROS_DOMAIN_ID"] = str(sys_id)
    
    micro_ros_node = Node(package = "micro_ros_agent",
                          name = "micro_ros",
                          executable = "micro_ros_agent",
                          arguments = ['--dev', '/dev/ttyUSB0', '-b', '921600'])

    bridge_node = Node(package = "bridging",
                       name = "bridge_node",
                       executable = "bridge",
                       parameters = [{'sys_id': sys_id, 'system_ids': ids}])
    
    pix_node = Node(package = 'pixhawk_interface',
                name = 'pixhawk_position_interface',
                executable = 'position',
                parameters = [{'sys_id': sys_id}])

    sys_node =  Node(package = 'system_interface',
                     name = 'system_interface',
                     executable = 'system',
                     parameters = [{'sys_id': sys_id, 'num_agents': len(ids)}])
    
    ld.add_action(micro_ros_node)
    ld.add_action(bridge_node)
    ld.add_action(pix_node)
    ld.add_action(sys_node)

    return ld
