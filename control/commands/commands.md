## Commands Package ##
A package with basic commands as ROS2 nodes, i.e. takeoff, land
As of right now this is just a start to familarize us with ignition gazebo, micro-ros-agent and ROS2 nodes.

To run the takeoff nodes use the command: `ros2 launch commands takeoff takeoff_alt:=<alt>`
where `<alt>` is your desired altitude. The default altitude is 10m.