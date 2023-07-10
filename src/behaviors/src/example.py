"""
Author: Jason Hughes
Date: July 2023
About: an example behavior
"""

from behavior_interface.behavior_ros_interface import BehaviorRosInterface
from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame

class BehaviorExample(BehaviorRosInterface):

    def __init__(self):
        super().__init__(NavMode.WAYPOINT, coordinate_frame = CoordinateFrame.LOCAL)

        self.desired_.position.x = 0.0
        self.desired_.position.y = 0.0
        self.desired_.position.z = 10.0
        
    def stepAutonomy(self):
        self.get_logger().info("Setting desired wp")
        self.desired_.position.x = 10.0
        self.desired_.position.y = 10.0
        self.desired_.position.z = 10.0



        
