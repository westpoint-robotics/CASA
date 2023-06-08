#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: May 2023
About: An implementation of distributed optimal transport (DOT) for task allocation in CASA
"""

import rclpy
import simplekml
import math
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from pykml import parser as kmlparser
from pykml.factory import nsmap
from geometry_msgs.msg import PoseStamped
from casa_msgs.msg import CasaPoseArray, CasaPoses
from sensor_msgs.msg import NavSatFix
from scipy.spatial.distance import cdist

from dot.llutmconversion import LLtoUTM
from dot.localpose import LocalPose
from dot.utmpose import UTMPose
from dot.planner import Planner


class DOTAllocator(Node):

    def __init__(self):

        super().__init__("Allocator")

        qos = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10 )
        
        self.declare_parameter("sys_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("threshold", rclpy.Parameter.Type.DOUBLE)
        self.sys_id_ = self.get_parameter("sys_id").value
        self.threshold_ = self.get_parameter("threshold").value

        topic_namespace = "casa"+str(self.sys_id_)
        
        self.local_pose_sub_ = self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                                        self.localCallback, qos) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, topic_namespace+"/internal/global_position",
                                                         self.globalCallback, qos)
        self.system_poses_ = self.create_subscription(CasaPoseArray, topic_namespace+"/internal/system_poses",
                                                      self.poseArrayCallback,
                                                      qos)

        self.task_publisher_ = self.create_publisher(NavSatFix,
                                                     topic_namespace+"/internal/task",
                                                     qos)
        
        self.timer_ = self.create_timer(0.1, self.cycleCallback)
        
        self.my_pose_ = LocalPose()
        self.my_utm_pose_ = (0,0)

        self.task_utm_poses_ = dict()
        self.sys_utm_poses_ = dict()
        self.task_locations_ = dict()
        
        self.at_task_ = False
        self.task_assigned_ = False
        self.task_ = 0
        self.task_number_ = None
        
        self.num_tasks_ = 0
        self.num_agents_ = 0
        
        self.planner_ = Planner()

        path = "/home/jason/casa_ws/src/control/dot/tasks/RiverCourtsTasks.kml"
        coords = self.loadTaskLocations(path)
        self.taskCoordsToUtm(coords)

        
    def localCallback(self, msg):
        self.my_pose_.x = msg.pose.position.x
        self.my_pose_.y = msg.pose.position.y
        self.my_pose_.z = msg.pose.position.z


    def globalCallback(self, msg):
        zone, e, n = LLtoUTM(23, msg.latitude, msg.longitude)
        self.my_utm_pose_ = (e,n)
        

    def poseArrayCallback(self, msg):
        i = 0
        utm_poses = dict()
        self.num_agents_ = 1
        for pose in msg.poses:
            lat = pose.global_pose.latitude
            lon = pose.global_pose.longitude
            (zone, east, north) = LLtoUTM(23, lat, lon)
            self.sys_utm_poses_[pose.sys_id] = (east, north)
            self.num_agents_ += 1
            
        
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
            self.get_logger().info("Loaded {} coordinates from tasks found in {}".format(len(coords),path))
        return coords


    def taskCoordsToUtm(self, coords):
        iter = 0
        for c in coords:
            lon = c[0]
            lat = c[1]
            (zone, easting, northing) = LLtoUTM(23, lat, lon)
            self.task_utm_poses_[iter] = (easting, northing)
            self.task_locations_[iter] = (lat, lon)
            iter += 1
            self.num_tasks_ += 1
        
    def publishTask(self, task, task_iter):
        msg = NavSatFix()
        msg.latitude = task[0]
        msg.longitude = task[1]
        msg.header.frame_id = "task: "+ str(task_iter)
        #msg.header.stamp = self.get_clock().now()

        self.task_publisher_.publish(msg)

        
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
        if not self.task_assigned_ and self.num_agents_ != 0:
            # optimize
            # create a distance matrix between all agents and all tasks
            self.sys_utm_poses_[self.sys_id_] = self.my_utm_pose_
            keys = list(self.sys_utm_poses_.keys())
            keys.sort()
            sorted_poses = {i: self.sys_utm_poses_[i] for i in keys}
            sys_matrix = np.array(list(sorted_poses.values()))
            task_matrix = np.array(list(self.task_utm_poses_.values()))

            #if you are at a task an going to another one, make sure you cant go to the task youre at
            if self.task_number_ != None:
                task_matrix[self.task_number_] = np.Inf
                
            dist = cdist(sys_matrix, task_matrix, 'euclidean')
            
            self.planner_.n = self.num_agents_
            self.planner_.m = self.num_tasks_
            self.planner_.p = 1.0
            self.planner_.dist = dist.flatten()
            self.planner_.pi = np.zeros((self.num_agents_, self.num_tasks_)).flatten()
            # call the optimization function
            out_matrix = self.planner_.optimize()
            task_weights = out_matrix[self.sys_id_-1,:]

            self.task_number_ = np.argmax(task_weights)
            
            self.task_ = self.task_locations_[self.task_number_]
            self.task_assigned_ = True
            self.get_logger().info("Agent %s assigned to task %s at %s " %(self.sys_id_, self.task_number_, self.task_) )

            self.publishTask(self.task_, self.task_number_) 
        else:
            if not self.at_task_ and self.task_assigned_:
                self.publishTask(self.task_, self.task_number_)
            else:
                self.task_assigned_ = False


"""
 TODOs
1. track agent ideas in list and organize the dictonaries in ascending order. 

"""


