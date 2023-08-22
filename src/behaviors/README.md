## Behaviors ##

In this module we provide an easy way to write behaviors for the swarm. The user can pick between two differnet navigation modes, waypoint and velocity. If they are using waypoint navigation you can specify between three coordinate frames, `LOCAL, GLOBAL, UTM` where global is latitude and longitude. This specification changes what `self.desired_` is, they become the logical ROS message, with the exception of utm which has a custom type since there is no ROS message. A table summarizes the options below.


| Nav Mode    | Coordinate Frame | desired type |
| ----------- | ---------------- |------------- |
| Velocity    | None             | Twist        |
| Waypoint    | Local            | Pose         |
| Waypoint    | Global           | NavSatFix    |
| Waypoint    | UTM              | utm_pose     |

### Running a Behavior ###

**As of August 2022**
To run a behavior, launch CASA for your agent or simulation first. Once CASA is running run: `ros2 launch behaviors behaviors.launch.py` or `ros2 launch behaviors behaviors_sim.launch.py num_agents:=X`. You will then be prompted with `$What behavior do you want to run: ` where you would enter the name of the behavior in the yaml file, (i.e. example).

### Adding a New Behavior ###
To write a new behavior, create a new python file in `behaviors/src` then you'll need to start with with this shell:

```python
import rclpy
from behavior_interface.behavior_ros_interface import BehaviorRosInterface
from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame

class ExampleBehavior(BehaviorRosInterface):

    def __init__(self):
        super().__init__(NavMode.WAYPOINT, coordinate_frame = CoordinateFrame.LOCAL)

    def stepAutonomy(self):
        pass
```
Change the inputs to the class inheritance to your desired nav mode and cooridinate frame.

Next you'll need to add your behavior to `behaviors/config/behaviors.yaml` following this format
```yaml
Unique-Behavior-Name:
  filename: <your-behavior-filename>.py
  class: <behaviors-classname>.py
  params:
    param1: a
    param2: b
    ...
```
In your yaml in put the first line should be a unique name for your behvior, `filename` is the name the python file you created in `src/`, `class` is the name of class you created in your behavior (i.e. `ExampleBehavior`) and finally you can list parameters under `params` that you want to pass into your behavior. They become ROS 2 parameters so get them the same way you would get any ROS 2 param. 