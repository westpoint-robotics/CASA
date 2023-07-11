"""
Author: Jason Hughes
Date: July 2023
About: an example behavior
"""
import rclpy
import numpy as np
from behavior_interface.behavior_ros_interface import BehaviorRosInterface
from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame

class CircleBehavior(BehaviorRosInterface):

    def __init__(self):
        super().__init__(NavMode.WAYPOINT, coordinate_frame = CoordinateFrame.LOCAL)

        self.declare_parameter("radius", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("spacing", rclpy.Parameter.Type.INTEGER)
        
        self.radius_ = self.get_parameter("radius").value
        self.spacing_ = self.get_parameter("spacing").value
        self.counter_ = 0

        self.x_ = np.array([0,0])
        self.y_ = np.array([0,0])

        self.generatePoints()
        
        self.desired_.position.x = 0.0
        self.desired_.position.y = 0.0
        self.desired_.position.z = 10.0

    def generatePoints(self):
        theta = np.linspace(0, 2*np.pi, self.spacing_)
        self.x_ = self.radius_ * np.cos(theta)
        self.y_ = self.radius_ * np.sin(theta)
        
    def stepAutonomy(self):

        if self.wp_distance_ < 2.0:
            if self.counter_ == self.spacing_-1:
                self.counter_ = 0
            else:
                self.counter_ += 1
            
        self.desired_.position.x = self.x_[self.counter_]
        self.desired_.position.y = self.y_[self.counter_]
        self.desired_.position.z = 10.0


        
