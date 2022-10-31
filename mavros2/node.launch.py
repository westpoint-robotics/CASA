import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration



def generate_launch_description():

    param = os.path.join(get_package_share_directory('mavros'),
                          'launch',
                          'ros2_param.yaml')
    
    mavros_node = Node(
        package = "mavros",
        executable = "mavros_node",
        parameters=[param]
        )

    return LaunchDescription([
        mavros_node
        ])
        
