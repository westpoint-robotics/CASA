## System Interface Package ##

This package is used to organize data coming in from the swarm and then pass that information onto the low level planner. At the heart of this package is the node `system.cpp`. This node subscribes to the gps position data of the other agents and uses the AgentTracker object to convert that into the local frame of the agent it is running on.
To run this package in simulation use: 'ros2 launch system_interface system_sim.launch.py num_agents:=XX`
To run this package on a companion computer use the command: `ros2 launch system_interface system.launch.py`