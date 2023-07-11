"""
Author: Jason Hughes
Date: July 2023
About: an example behavior
"""
import rclpy
from behavior_interface.behavior_ros_interface import BehaviorRosInterface
from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame

class GoToXY(BehaviorRosInterface):

    def __init__(self):
        super().__init__(NavMode.WAYPOINT, coordinate_frame = CoordinateFrame.LOCAL)

        self.declare_parameter("x", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("y", rclpy.Parameter.Type.DOUBLE)

        self.x_ = self.get_parameter("x").value
        self.y_ = self.get_parameter("y").value
        
        self.desired_.position.x = self.x_
        self.desired_.position.y = self.y_
        self.desired_.position.z = 10.0
        
    def stepAutonomy(self):
    
        # set your desired waypoint with the self.desired_ variable 
        self.desired_.position.x = self.x_
        self.desired_.position.y = self.y_
        self.desired_.position.z = 10.0

        if self.wp_distance_ < 2.0:
            self.get_logger().info("Reached local waypoint")


        
