#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: May 2023
About: An implementation of distributed optimal transport (DOT) for task allocation in CASA
"""

import os
import rclpy
import simplekml
import math
import time
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from pykml import parser as kmlparser
from pykml.factory import nsmap
from geometry_msgs.msg import PoseStamped
from casa_msgs.msg import CasaPoseArray, CasaPoses
from casa_msgs.msg import CasaTaskArray, CasaTask
from std_msgs.msg import Int32
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
        self.declare_parameter("path", rclpy.Parameter.Type.STRING)
        self.sys_id_ = self.get_parameter("sys_id").value
        self.threshold_ = self.get_parameter("threshold").value
        self.path_ = self.get_parameter("path").value

        topic_namespace = "casa"+str(self.sys_id_)
        
        self.local_pose_sub_ = self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                                        self.localCallback, qos) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, topic_namespace+"/internal/global_position",
                                                         self.globalCallback, qos)
        self.system_poses_ = self.create_subscription(CasaPoseArray, topic_namespace+"/internal/system_poses",
                                                      self.poseArrayCallback,
                                                      qos)

        self.system_tasks_sub = self.create_subscription(CasaTaskArray, topic_namespace+"/internal/system_tasks",
                                                      self.taskArrayCallback,
                                                      qos)
        
        self.task_publisher_ = self.create_publisher(PoseStamped,
                                                     topic_namespace+"/internal/task",
                                                     qos)

        self.task_iter_publisher_ = self.create_publisher(Int32,
                                                          topic_namespace+"/internal/task_number",
                                                          qos)
        
        self.timer_ = self.create_timer(0.1, self.cycleCallback)
        
        self.my_pose_ = LocalPose()
        self.my_utm_pose_ = (0,0)

        self.task_utm_poses_ = list()
        self.task_local_poses_ = list()
        self.completed_tasks_ = list()
        self.sys_utm_poses_ = dict()
        self.system_tasks_ = dict()
        self.task_locations_ = dict()
        self.task_x_ = None
        self.task_y_ = None
        
        self.at_task_ = False
        self.task_assigned_ = False
        self.first_assign_ = False
        self.got_my_pose_ = False
        self.going_home_ = False
        self.task_ = 0
        self.task_number_ = None
        self.easting0 = 0
        self.northing0 = 0

        self.master_tasks_ = dict()
        
        self.num_tasks_ = 0
        self.num_agents_ = 0
        self.counter_ = 0
        
        self.planner_ = Planner()

        self.loaded_files_ = list()
        user = os.getlogin()
        self.path_ = "/home/" + user + "/"+self.path_
        #coords = self.loadTaskLocations()
        #self.taskCoordsToUtmAndLocal(coords)
        #self.assignTask()
        
        
    def localCallback(self, msg):
        self.my_pose_.x = msg.pose.position.x
        self.my_pose_.y = msg.pose.position.y
        self.my_pose_.z = msg.pose.position.z


    def globalCallback(self, msg):
        zone, e, n = LLtoUTM(23, msg.latitude, msg.longitude)
        self.got_my_pose_ = True
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
        # self.get_logger().info("Loaded {} agents ".format(self.num_agents_))


    def taskArrayCallback(self, msg):
        for task in msg.system_tasks:
            self.system_tasks_[task.sys_id] = task.assigned_task
        #self.deleteTasks()
            
            
    def globalToLocal(self, easting, northing):
        # convert from utm to local frame in 2D
        if self.counter_ < 1:
            self.easting0 = self.my_utm_pose_[0] - self.my_pose_.x
            self.northing0 = self.my_utm_pose_[1] - self.my_pose_.y
        local_x = easting - self.easting0
        local_y = northing - self.northing0

        return local_x, local_y
    
        
    def loadTaskPlacemarkers(self, placemarkers):    
        coords = []
        count = 0
        for p in placemarkers:
            if hasattr(p , 'Point'):
                coords_str = str(p.Point.coordinates)
            else:
                self.get_logger().error("Didn't find a point for {} kml placemarker.  Not loading! Check README.".format("BHG Tasks"))
                return coords
            coords_str.replace("\t","")
            coords_str.replace("\n","")
            coords_temp = coords_str.split(" ")
            for cstr in coords_temp:                    
                try:
                    count += 1
                    k = [float(s) for s in cstr.split(",")]
                    coords.append(k)                    
                except ValueError:                    
                    pass
        return coords

    
    def loadTaskLocations(self):
        coords = []
        files = os.listdir(self.path_)
        for f in files:
            if not f in self.loaded_files_:
                with open(self.path_+f) as kml_file:
                    root = kmlparser.parse(kml_file).getroot().Document
                    placemarkers = root.Folder.Placemark
                    new_coords = self.loadTaskPlacemarkers(placemarkers)
                    coords = coords + new_coords
                    self.get_logger().info("Loaded {} coordinates from tasks found in {}".format(len(new_coords),f))
            self.loaded_files_.append(f)
        return coords


    def taskCoordsToUtmAndLocal(self, coords):
        # convert task coords from lat lon to utm
        for c in coords:
            lon = c[0]
            lat = c[1]
            (zone, easting, northing) = LLtoUTM(23, lat, lon)
            self.task_utm_poses_.append((easting, northing))
            x,y = self.globalToLocal(easting,northing)
            self.task_local_poses_.append((x,y))
            self.task_locations_[self.num_tasks_] = (lat, lon)
            self.master_tasks_[self.num_tasks_] = (x,y)
            self.num_tasks_ += 1
        # log_message = "Contents of self.master_tasks_: %s" % self.master_tasks_
        # self.get_logger().info(log_message)

    def publishTask(self, task, task_iter):
        # publish the task coordinates in the local frame 
        msg = PoseStamped()
        msg.pose.position.x = task[0]
        msg.pose.position.y = task[1]
        msg.header.frame_id = "task: "+ str(task_iter)

        self.task_publisher_.publish(msg)


    def publishHome(self):
        # function to publish (0,0) when agent needs to go home
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.header.frame_id = "home"

        self.task_publisher_.publish(msg)

        
    def publishTaskIter(self):
        msg = Int32()
        msg.data = self.getTaskIter((self.task_y_, self.task_x_))
        
        self.task_iter_publisher_.publish(msg)
        
        
    def feedbackLoop(self, task):
        #function to check if the agent is near its
        current_pose = [self.my_pose_.x, self.my_pose_y]
        task_loc = self.task_locations_[task]
        task_loc = [task_loc.x, task_loc.y]

        #compare distance
        distance = math.dist(current_pose, task_loc)

        if (distance < threshold):
            self.at_task_ = True

            
    def checkThreshold(self):
        # check if the agent is within the threshold of its task
        if self.task_x_ != None:
            d = math.dist((self.my_pose_.x, self.my_pose_.y),(self.task_x_, self.task_y_))
            return d < self.threshold_
        else:
            return False


    def getTaskIter(self, ind):
        return list(self.master_tasks_.keys())[list(self.master_tasks_.values()).index(ind)]

    
    def deleteSysTasks(self):
        for ag,task in zip(self.system_tasks_.keys(), self.system_tasks_.values()):
            task_local = self.master_tasks_[task]
            print(task_local)
            try:
                #get the index if it exists
                ind = self.task_local_poses_.index(task_local)
            except ValueError as e:
                ind = len(self.master_tasks_) + 1

            if not (task in self.completed_tasks_):
                # self.get_logger().info("agent %s sees agent %s going to task %s, deleting" %(self.sys_id_, ag, task))
                self.task_local_poses_.pop(ind)
                self.task_utm_poses_.pop(ind)
                self.completed_tasks_.append(task)
            
                
    def assignTask(self):
        # optimize                                                                                   
        # create a distance matrix between all agents and all tasks                                 
        self.first_assign_ = True

        if len(self.task_utm_poses_) == 0:
            self.get_logger().info("No tasks in queue, returning home")
            self.going_home_ = True
            return 0
        
        #organize the poses of agents and tasks
        self.sys_utm_poses_[self.sys_id_] = self.my_utm_pose_                                       
        keys = list(self.sys_utm_poses_.keys())                                                     
        keys.sort()                                                                                 
        sorted_poses = {i: self.sys_utm_poses_[i] for i in keys}                                    
        sys_matrix = np.array(list(sorted_poses.values()))                                          
        task_matrix = np.array(self.task_utm_poses_)

        # calc the euclidean distance matrix
        dist = cdist(sys_matrix, task_matrix, 'euclidean')                                          

        # set the optimization parameters
        self.planner_.n = self.num_agents_                                                          
        self.planner_.m = len(self.task_utm_poses_)                                                 
        self.planner_.p = 1.0                                                                       
        self.planner_.dist = dist.flatten()                                                         
        self.planner_.pi = np.zeros((self.planner_.n, self.planner_.m)).flatten()                   

        # call the optimization function                                                            
        out_matrix = self.planner_.optimize()                                                       
        task_weights = out_matrix[self.sys_id_-1,:]                                                 

        # interperet the output of the optimizer
        self.task_number_ = np.argmax(task_weights)                                                
        self.task_ = self.task_locations_[self.task_number_]                                        
        task_local = self.task_local_poses_[self.task_number_]
        self.task_y_, self.task_x_ = task_local[0], task_local[1]

        master_task_index = self.getTaskIter(task_local) 
        self.completed_tasks_.append(master_task_index)
        #inform and publish
        self.get_logger().info("Agent %s assigned to task %s located at %s " %(self.sys_id_, master_task_index, task_local))
        
        self.publishTask((self.task_x_,self.task_y_), self.task_number_)                           
        self.publishTaskIter()
        
        #remove the task we just assigned
        self.task_utm_poses_.pop(self.task_number_)
        self.task_local_poses_.pop(self.task_number_)
        
        
    def cycleCallback(self):
        # calc distance between myself and all tasks
        # take argmin of matrix
        
        if self.going_home_: #or len(self.task_utm_poses_)< self.num_agents_ and self.counter_ > 0:
            self.publishHome()
            return 0
        
        if self.checkThreshold() and self.num_agents_ > 0 and self.my_utm_pose_[0] != 0:
            if self.counter_ < 1:
                coords = self.loadTaskLocations()
                self.taskCoordsToUtmAndLocal(coords)
                self.assignTask()
                self.counter_ += 1
            else:
                self.get_logger().info("At task location, completeing task")
                time.sleep(1.0)
                self.get_logger().info("Task completed, assigning new task")
                self.deleteSysTasks()
                self.assignTask()
                self.counter_ += 1
        elif self.num_agents_ > 0 and self.my_utm_pose_[0] != 0:
            if self.first_assign_:
                self.publishTask((self.task_x_,self.task_y_), self.task_number_)
                self.publishTaskIter()
                self.deleteSysTasks()
            else:
                if self.counter_ < 1:
                    coords = self.loadTaskLocations()
                    self.taskCoordsToUtmAndLocal(coords)
                    self.assignTask()
                    self.counter_ += 1
        
        if self.counter_ >= 1:
            coords = self.loadTaskLocations()
            self.taskCoordsToUtmAndLocal(coords)


            
        # TODO:
        # 1. error handling if no tasks in queue -- DONE
        # 2. handling uploading multiple tasks -- DONE
        # 3. Wait at task for 3 seconds before assigning next one -- DONE
        # 4. more than two agents -- DONE
        # 5. 

