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
        # define the navigation mode (velocity or waypoint)
        # if using waypoint navigation define the coordinframe (local, global or utm)
        super().__init__(NavMode.WAYPOINT, coordinate_frame = CoordinateFrame.LOCAL)

        self.desired_.position.x = 0.0
        self.desired_.position.y = 0.0
        self.desired_.position.z = 0.0
        
    def stepAutonomy(self):
        # step autonomy function called at 10Hz
        self.get_logger().info("Setting desired wp")

        # set your desired waypoint with the self.desired_ variable 
        self.desired_.position.x = 10.0
        self.desired_.position.y = 10.0
        self.desired_.position.z = 10.0

        # check how close you are to waypoint with the self.wp_distance_ variable
        if self.wp_distance_ < 1.0:
            pass

        #check my position
        my_lat = self.global_pose_.latitude
        my_lon = self.global_pose_.longitude
        my_alt = self.global_pose_.altitude

        my_easting = self.global_pose_.easting
        my_northing = self.global_pose_.northing

        my_local_x = self.local_pose_.x
        my_local_y = self.local_pose_.y
        my_local_z = self.local_pose_.z

        # use the self.system_ dictionary to see other agent information
        for (sys_id, agent_info) in self.system_.items():
            unique_id = sys_id
            
            latitude = agent_info.global_pose.latitude
            longitude = agent_info.global_pose.longitude

            altitude = agent_info.global_pose.altitude

            utm_easting = agent_info.global_pose.easting
            utm_northing = agent_info.global_pose.northing

            local_pose_x = agent_info.local_pose.x
            local_pose_y = agent_info.local_pose.y
            local_pose_z = agent_info.local_pose.z

            heading = agent_info.heading
            
            assigned_task = agent_info.assigned_task
            connectivity_level = agent_info.connectivity_level


        
