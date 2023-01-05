## Multi Agent Simulation ##

First, make sure ignition gazebo is installed, follow the instructions [here](https://gazebosim.org/docs/garden/install_ubuntu)

In the `PX4_Autopilot` directory run:
```
make px4_sitl
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0"  PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1
```

In another terminal run:
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0"  PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 2
```
And so on until you have the desired amount of agents.
Finally in another terminal run:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
You should see the following nodes
```
$ ros2 node list
/px4_1/px4_micro_xrce_dds
/px4_2/px4_micro_xrce_dds
```

You can set the names space with the `PX4_MICRODDS_NS` environment variable. 