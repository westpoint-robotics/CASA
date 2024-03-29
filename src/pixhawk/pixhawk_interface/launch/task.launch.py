import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, AnonName


def generate_launch_description():

    ld = LaunchDescription()

    for arg in sys.argv:
        if arg.startswith("num_agents:="):
            num_agents = int(arg.split(":=")[1])

    for i in range(1,num_agents+1):
        pix_node = Node(package = 'pixhawk_interface',
                    name = 'casa_'+str(i)+'_pixhawk_task',
                    executable = 'task_goto',
                    parameters = [{'sys_id': i, 'altitude': 10.1}])

        ld.add_action(pix_node)       
        
    return ld
