from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    takeoff_alt = DeclareLaunchArgument('takeoff_alt', default_value='10')

    node = Node(package='commands',
                executable='takeoff',
                name='takeoff_node',
                parameters=[{"takeoff_alt": LaunchConfiguration('takeoff_alt')}]
                )
    
    return LaunchDescription([takeoff_alt, node])
