## Pixhawk Interface Package ##

This package is used to house nodes that interface with the pixhawk.

-`position.cpp` relays local and global position up to casa
-`status.cpp` relays failsafe information from the pixhawk up to casa

To launch these node for simulation use `ros2 launch pixhawk_interface pixhawk_sim.lauuch.py num_gents:=X`. This will launch a set of nodes for each simulated pixhawk agent.

To launch this on a companion computer make sure the `SYS_ID` environment variable is exported and use the command: `ros2 launch pixhawk_interface pixhawk.launch.py`.