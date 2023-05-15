#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: May 2023
About: An implementation of distributed optimal transport (DOT) for task allocation in CASA
"""

import rclpy
import simplekml

from rclpy.node import Node
from pykml import parser as kmlparser
from pykml.factory import nsmap
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from scipy.spatial.distance import cdist

from dot.llutmconversion import LLtoUTM
from dot.localpose import LocalPose
from dat.utmpose import UTMPose


class DOTAllocator(Node):

    def __init__(self):

        super().__init__("Allocator")

        self.local_pose_sub_ = self.create_subscription(PoseStamped, "/internal/local_position", self.poseCallnack, 10) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, "/internal/global_position", self.globalCallback, 10)
        
        self.my_pose_ = LocalPose()
        self.my_utm_pose_ = UTMPose()
        self.tasks_locations_ = list()

        coords = self.loadTaskLocations(path)
        globalToLocal(coords)

        
    def localCallback(self, msg):
        self.my_pose_.x = msg.pose.position.x
        self.my_pose_.y = msg.pose.position.y
        self.my_pose_.z = msg.pose.position.z


    def globalCallback(self, msg):
        self.my_utm_pose_.zone, self.my_utm_pose_.easting, self.my_utm_pose_.northing = LLtoUTM(23, msg.latitude, msg.longitude)
        
        
    def loadTaskPlacemarkers(self, placemarkers):    
        coords = []
        for p in placemarkers:
            if hasattr(p , 'Point'):
                coords_str = str(p.Point.coordinates)
            else:
                self.get_logger().error("Didn't find a point for {} kml placemarker.  Not loading! Check README.".format("rivercourts"))
                return coords
            coords_str.replace("\t","")
            coords_str.replace("\n","")
            coords_temp = coords_str.split(" ")
            for cstr in coords_temp:                    
                try:                        
                    k = [float(s) for s in cstr.split(",")]
                    coords.append(k)                    
                except ValueError:                    
                    pass
        return coords

    
    def loadTaskLocations(self, path):
        coords = []
        with open(path) as kml_file:
            root = kmlparser.parse(kml_file).getroot().Document
            placemarkers = root.Folder.Placemark
            coords = self.loadTaskPlacemarkers(placemarkers)
            self.get_logger().info("Loaded {} coordinates from plume found in {}".format(len(coords),path))
        return coords


    def globalToLocal(self, coords):
        
        for c in coords:
            p = LocalPose()
            lon = c[0]
            lat = c[1]
            (zone, easting, northing) = LLtoUTM(23, lat, lon)
            easting_diff = self.my_utm_pose_.easting - easting
            northing_diff = self.my_utm_pose_.northing - northing
            p.x = self.my_pose_.x + easting_diff
            p.y = self.my_pose_.y + northing_diff
            p.z = c[2]
            self.task_locations_.append(p)

            
                                               
if __name__ == "__main__":
    rclpy.init()

    DOTAllocator()
