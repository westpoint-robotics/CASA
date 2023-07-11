"""
Author: Jason Hughes
Date: July 2023
About: Behavior organization node
"""
import copy
import math
import rclpy

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from casa_msgs.msg import CasaAgent, CasaAgentArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame
from behavior_interface.local_pose import LocalPose
from behavior_interface.global_pose import GlobalPose
from behavior_interface.utm_pose import UtmPose
from behavior_interface.ll_utm_conversion import LLtoUTM, UTMtoLL


class BehaviorRosInterface(Node):

    def __init__(self, nav_mode, coordinate_frame=CoordinateFrame.NONE):

        super().__init__("BehaviorRosInterface")

        qos = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10 )

        self.nav_mode_ = nav_mode
        self.coordinate_frame_ = coordinate_frame

        self.declare_parameter("sys_id", rclpy.Parameter.Type.INTEGER)
        self.sys_id_ = self.get_parameter("sys_id").value

        topic_namespace = "casa"+str(self.sys_id_)

        # subscriptions
        self.local_pose_sub_ = self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                                        self.localCallback, qos) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, topic_namespace+"/internal/global_position",
                                                         self.globalCallback, qos)
        self.system_poses_ = self.create_subscription(CasaAgentArray, topic_namespace+"/internal/system_array",
                                                      self.systemArrayCallback,
                                                      qos)

        # timer
        self.timer_ = self.create_timer(0.1, self.cycleCallback)
        self.step_autonomy_ = self.create_timer(0.1, self.stepAutonomy)

        self.heading_pub_ = self.create_publisher(Float32,
                                                  topic_namespace+"/internal/goto_heading",
                                                  qos)
                                                  
        
        if self.nav_mode_ == NavMode.VELOCITY:
            self.desired_ = Twist()
            self.desired_pub_ = self.create_publisher(TwistStamped,
                                                      topic_namespace+"/internal/goto_vel",
                                                      qos)
            self.coordinate_frame_ = CoordinateFrame.NONE
        elif self.nav_mode_ == NavMode.WAYPOINT:
            self.desired_pub_ = self.create_publisher(PoseStamped,
                                                      topic_namespace+"/internal/goto_pose",
                                                      qos)
            if self.coordinate_frame_ == CoordinateFrame.LOCAL:
                self.desired_ = Pose()
            elif self.coordinate_frame_ == CoordinateFrame.GLOBAL:
                self.desired_ = NavSatFix()
            elif self.coordinate_frame_ == CoordinateFrame.UTM:
                self.desired_ = UtmPose()
            else:
                self.get_logger().info("Incorrect Coordinate Frame set, the options are LOCAL, GLOBAL, and UTM")
        else:
            self.get_logger().info("Incorrect Navigation Mode set, the options are VELOCITY and WAYPOINT")

        # variables to look for 
        self.local_pose_ = LocalPose()
        self.global_pose_ = GlobalPose()
        self.system_ = dict()
        self.wp_distance_ = 0.0

        self.heading_ = None
        
        self.zone_ = None

        self.easting_origin_ = 0.0
        self.northing_origin_ = 0.0
        self.local_x_0_ = 0.0
        self.local_y_0_ = 0.0
        
        self.utm_pose_ = (0.0,0.0)

        self.global_pose_counter_ = 0
        self.local_pose_counter_ = 0
        
        
    def localCallback(self, msg):
        self.local_pose_.initializeFromPoint(msg.pose.position)
        if self.local_pose_counter_ == 0:
            self.local_x_0_ = msg.pose.position.x
            self.local_y_0_ = msg.pose.position.y
        self.local_pose_counter_ += 1
        
        
    def globalCallback(self, msg):
        self.global_pose_.initializeFromNavSatFix(msg)
        self.utm_pose_ = (self.global_pose_.easting, self.global_pose_.northing)
        if self.global_pose_counter_ == 0:
            self.easting_0_ = copy.copy(self.utm_pose_[0])
            self.northing_0_ = copy.copy(self.utm_pose_[1])
        self.global_pose_counter_ += 1
        
    def systemArrayCallback(self, msg):
        for ag in msg.agents:
            self.system_[ag.sys_id] = ag

    def calcDistance(self):
        if self.coordinate_frame_ == CoordinateFrame.LOCAL:
            my_pose = (self.local_pose_.x, self.local_pose_.y)
            wp_pose = (self.desired_.position.x, self.desired_.position.y) 
            return math.dist(my_pose, wp_pose)
        elif self.coordinate_frame_ == CoordinateFrame.UTM:
            my_pose = self.utm_pose_
            wp_pose = (self.desired_.easting, self.desired_.northing)
            return math.dist(my_pose, wp_pose)
        elif self.coordinate_frame_ == CoordinateFrame.GLOBAL:
            z, d_e, d_n = LLtoUTM(23, self.desired_.latitude, self.desired_.longitude)
            my_pose = self.utm_pose_
            wp_pose = (d_e, d_n)
            return math.dist(my_pose, wp_pose)

        
    def publishDesired(self, msg, frame):
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = frame
        self.desired_pub_.publish(msg)

        
    def globalToLocal(self, easting, northing):
        local_x = easting - self.easting_origin_
        local_y = northing - self.northing_origin_

        return local_x, local_y

    def calculateHeading(self, desired_x, desired_y):
        heading = math.atan2(desired_y - self.local_pose_.y, desired_x - self.local_pose_.x)
        return heading
    
    def cycleCallback(self):
        self.get_logger().info("publishing...")
        
        if self.nav_mode_ == NavMode.VELOCITY:
            msg = TwistStamped()
            msg.twist = self.desired_
            self.publishDesired(msg, "local_vel")

        elif self.nav_mode_ == NavMode.WAYPOINT:

            self.wp_distance_ = self.calcDistance()

            if self.coordinate_frame_ == CoordinateFrame.LOCAL:
                msg = PoseStamped()
                msg.pose = self.desired_

                if self.heading_ != None:
                    msg_h = Float32()
                    msg_h.data = self.heading_
                    self.heading_pub_.publish(msg_h)
                else:
                    msg_h = Float32()
                    msg_h.data = self.calculateHeading(self.desired_.position.x, self.desired_.position.y)
                    self.heading_pub_.publish(msg_h)

                self.publishDesired(msg, "local")

            elif self.coordinate_frame_ == CoordinateFrame.GLOBAL:
                z, e, n = LLtoUTM(23, self.desired_.latitude, self.desired_.longitude)
                x, y = self.globalToLocal(e, n)
                msg = PoseStamped()
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = self.desired_.altitude

                if self.heading_ != None:
                    msg_h = Float32()
                    msg_h.data = self.heading_
                    self.heading_pub_.publish(msg)
                else:
                    msg_h = Float32()
                    msg_h.data = self.calculateHeading(x,y)
                    self.heading_pub_.publish(msg) 

                self.publishDesired(msg, "local")
                
            elif self.coordinate_frame_ == CoordinateFrame.UTM:
                x, y = self.globalToLocal(self.desired_.easting, self.desired_.northing)
                msg = PoseStamped()
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = self.desired_.altitude

                if self.heading_ != None:
                    msg_h = Float32()
                    msg_h.data = self.heading_
                    self.heading_pub_.publish(msg_h)
                else:
                    msg_h = Float32()
                    msg_h.data = self.calculateHeading(x,y)
                    self.heading_pub_.publish(msg_h)
                    
                self.publishDesired(msg, "local")
        
        
