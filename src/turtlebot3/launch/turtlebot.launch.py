import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace 
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    sys_id = int(os.getenv('ROS_DOMAIN_ID'))
    
    node1 = Node(package = 'turtlebot3',
                name = "turtlebot_"+str(sys_id)+"_node",
                executable = "turtlebot_interface",
                parameters = [{'sys_id': sys_id}])

    ld.add_action(node1)

    return ld
