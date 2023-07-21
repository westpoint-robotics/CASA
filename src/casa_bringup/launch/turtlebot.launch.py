import os
import sys

from casa_bringup.network import scan  

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

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
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"
    
    bot = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                                      get_package_share_directory("turtlebot3_bringup"),'launch/robot.launch.py')))

    sys_node =  Node(package = 'system_interface',
                     name = 'casa_'+str(sys_id)+'_system_interface',
                     executable = 'system',
                     parameters = [{'sys_id': sys_id, 'num_agents': len(ids)}])

    bridge_node = Node(package = "bridging",
                       name = "casa_"+str(sys_id)+"_topic_bridge",
                       executable = "bridge",
                       parameters = [{'sys_id': sys_id, 'system_ids': ids}])
    
    
    casa_bot_node = Node(package = 'turtlebot3_interface',
                         name = "turtlebot_"+str(sys_id)+"_node",
                         executable = "turtlebot_interface",
                         parameters = [{'sys_id': sys_id}])

    return LaunchDescription([bot,
                              bridge_node,
                              sys_node,
                              casa_bot_node])
