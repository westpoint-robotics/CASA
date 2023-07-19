#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ld = LaunchDescription()
    
    for arg in sys.argv:
        if arg.startswith("num_agents:="):
            num_agents = int(arg.split(":=")[1])
    
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    
    for i in range(1,num_agents+1):

        nmspc = DeclareLaunchArgument("bot_"+str(i),
                                      default_value=TextSubstitution(text="bot_"+str(i)))

        ld.add_action(nmspc)
        
        x_pose = LaunchConfiguration('x_pose', default=str(float(i)))
        y_pose = LaunchConfiguration('y_pose', default='0.0')
        
        robot_state_publisher_cmd = GroupAction(actions=[PushRosNamespace(LaunchConfiguration("bot_"+str(i))),
                                                         IncludeLaunchDescription(
                                                             PythonLaunchDescriptionSource(
                                                             os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                                                             ),
                                                             launch_arguments={'use_sim_time': use_sim_time}.items())])

        spawn_turtlebot_cmd = GroupAction(actions=[PushRosNamespace(LaunchConfiguration("bot_"+str(i))),
                                                   IncludeLaunchDescription(
                                                       PythonLaunchDescriptionSource(
                                                           os.path.join(launch_file_dir,'spawn_turtlebot3.launch.py')
                                                       ),
                                                       launch_arguments={
                                                           'x_pose': x_pose,
                                                           'y_pose': y_pose
                                                       }.items())])

        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)

    return ld
