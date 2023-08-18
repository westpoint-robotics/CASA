## CASA Bringup ##

An example of launching for a turtlebot in hardware would be:
```
ros2 launch casa_bringup turtlebot.launch.py name:=ubunut.net,plato.net network:=wlo1
```
An example of launching casa after bringing up a px4 gazebo simulation would be:
```
ros2 launch casa_bringup pixhawk_sim.launch.py num_agents:=3
```

#### Launch Files ####

- `pixhawk.launch.py`: For running casa on hardware with a pixhawk running PX4 DDS firmware.
- `single_turtlebot_sim.launch.py`: launch a turtlebot simulation with one turtlebot and casa.
- `turtlebot.launch.py`: launch necessary turtlebot nodes and casa for hardware.
- `pixhawk_sim.launch.py`: launch a pixhawk simulation

There are two inputs allowed for the hardware launch files. The first is name, this is the name of the device associated with the IP address, the default is `ubuntu.net` for the rpi's. You can add names like the name of a groundstation computer. Also you can specify the network interface, defualt is `wlo1`. 

#### Network Start Up ####
The lunach files calls helper functions kept in `network.py`, these functions allow scannig of the network to see what other IP addresses are active on the network. This allows use to see what other agents are on the network and let the bridging node know so that it can create a bridge between the two IDs. It also returns your own IP address. The last octet of the IP is used as the `ROS_DOMAIN_ID`, this guarentees that each domain ID is unique. 