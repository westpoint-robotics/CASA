#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: May 2023
About: An implementation of distributed optimal transport (DOT) for task allocation in CASA
"""

import rclpy
import simplekml
import math

from rclpy.node import Node
from pykml import parser as kmlparser
from pykml.factory import nsmap
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import NavSatFix
from scipy.spatial.distance import cdist

from dot.llutmconversion import LLtoUTM
from dot.localpose import LocalPose
from dot.utmpose import UTMPose
from dot.planner import Planner


class DOTAllocator(Node):

    def __init__(self):

        super().__init__("Allocator")
        
        self.declare_parameter("sys_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("threshold", rclpy.Parameter.Type.FLOAT)
        self.sys_id_ = self.get_parameter("sys_id")
        self.threshold_ = self.get_parameter("threshold")

        topic_namespace = "casa"+str(sys_id)
        
        self.local_pose_sub_ = self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                                        self.poseCallback, 10) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, topic_namespace+"/internal/global_position",
                                                         self.globalCallback, 10)
        self.system_poses_ = self.create_subscription(PoseArray, topic_namespace+"/internal/system_poses",
                                                      self.poseArrayCallback,
                                                      10)

        self.task_publisher_ = self.create_publisher(topic_namespace+"/internal/task",
                                                     PoseStamped,
                                                     10)
        
        self.timer_ = self.create_timer(0.1, self.cycleCallback)
        
        self.my_pose_ = LocalPose()
        self.my_utm_pose_ = UTMPose()

        self.task_poses_ = list()
        self.sys_poses_ = list()
        self.sys_poses_2D_ = np.array()
        self.task_poses_2D_ = np.array()
        
        self.at_task_ = False
        self.task_assigned_ = False

        self.num_tasks_ = 0
        self.num_agents_ = 0
        
        self.planner_ = Planner()
        
        coords = self.loadTaskLocations(path)
        globalToLocal(coords)

        
    def localCallback(self, msg):
        self.my_pose_.x = msg.pose.position.x
        self.my_pose_.y = msg.pose.position.y
        self.my_pose_.z = msg.pose.position.z


    def globalCallback(self, msg):
        self.my_utm_pose_.zone, self.my_utm_pose_.easting, self.my_utm_pose_.northing = LLtoUTM(23, msg.latitude, msg.longitude)


    def poseArrayCallback(self, msg):
        i = 0
        poses = list()
        for pose in msg.poses:
            i += 1
            self.sys_poses_.append(initializeFromPoint(pose.position))
            poses.append((pose.position.x, pose.position.y))
            #set the number of agents in the swarm for the planner
        self.sys_poses_2D_ = np.array(poses)
        self.num_agents_ = i
            
        
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
        loc_list = list()
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
            loc_list.append([p.x,p.y])
            self.task_poses_.append(p)
        self.task_poses_2D_ = np.array(loc_list)
        # set the number of tasks to complete
        self.num_tasks_ = len(loc_list) 
        loc_list.clear()

        
    def publishTask(self, task_index):
        task_loc = self.task_locations_[task_index]
        msg = PoseStamped()
        msg.pose.position.x = task_loc.x
        msg.pose.position.y = task_loc.y
        msg.header.frame_id = "task: " + str(task_index)
        msg.header.stamp = self.get_clock().now()

        self.task_publisher_.publish(msg)

        
    def publishBlankTask(self):
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.header.frame_id = "no task assigned"
        msg.header.stamp = self.get_clock().now()
        

    def feedbackLoop(self, task):
        #function to check if the agent is near its
        current_pose = [self.my_pose_.x, self.my_pose_y]
        task_loc = self.task_locations_[task]
        task_loc = [task_loc.x, task_loc.y]

        #compare distance
        distance = math.dist(current_pose, task_loc)

        if (distance < threshold):
            self.at_task_ = True
            
            
            
    def cycleCallback(self):
        # calc distance between myself and all tasks
        # take argmin of matrix
        if not self.task_assigned_:
            # optimize
            # create a distance matrix between all agents and all tasks
            dist = cdist(self.sys_poses_2D_, self.task_poses_2D_, 'euclidean')
            #set properties in planner
            self.planner_.n = self.num_agents_
            self.planner_.m = self.num_tasks_
            self.planner_.dist = dist.flatten()
            self.planner_.pi = np.zeros(self.num_agents_, self.num_tasks_).flatten()
            # call the optimization function
            out_matrix = self.planner_.optimize()
            task_wieghts = out_matrix[self.sys_id_,:]
            task_number = np.argmax(task_weights)
            goto_task_pose = self.task_poses_2D_[task_number]
        else:
            if not self.at_task_:
                self.publishTask(task)
            else:
                self.publishBlankTask()
            
        

        
                                               
if __name__ == "__main__":
    rclpy.init()

    DOTAllocator()
