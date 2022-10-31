## MAVROS2 Support Files ##

MAVROS2 does not come with launch files or parameter files that are compatible with ROS2.
Copy the two files in here to `/opt/ros/humble/share/mavros/launch/` then use the command `ros2 launch mavros node.launch.py`

#### Installing MAVROS2 ####
To install mavros for ROS2 Humble use `sudo apt-get install ros-humble-mavros ros-humble-mavros-extras`. Then un the command `ros2 run mavros install install_geographic_datasets.sh`.