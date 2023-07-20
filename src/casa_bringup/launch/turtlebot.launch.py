import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    sys_id = int(os.getenv('ROS_DOMAIN_ID'))
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"
    
    bot_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                                      get_package_share_directory("turtlebot3_bringup"),'launch/robot.launch.py')))

    sys_node =  Node(package = 'system_interface',
                     name = 'casa_'+str(sys_id)+'_system_interface',
                     executable = 'system',
                     parameters = [{'sys_id': sys_id}])

    casa_bot_node = Node(package = 'turtlebot3_interface',
                         name = "turtlebot_"+str(sys_id)+"_node",
                         executable = "turtlebot_interface",
                         parameters = [{'sys_id': sys_id}])

    return LaunchDescription([bot_sim,
                              sys_node,
                              casa_bot_node])
