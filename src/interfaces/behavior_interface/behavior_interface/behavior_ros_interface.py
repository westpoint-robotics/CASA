"""
Author: Jason Hughes
Date: July 2023
About: Behavior organization node
"""
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from casa_msgs.msg import CasaAgent, CasaAgentArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist
from sensor_msgs.msg import NavSatFix

from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame
from behavior_interface.localpose import LocalPose
from behavior_interface.globalpose import GlobalPose
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
                                            
        if self.nav_mode_ == NavMode.VELOCITY:
            self.desired_ = Twist()
            self.desired_pub_ = self.create_publisher(TwistStamped,
                                                      topic_namespace+"/internal/behavior_vel",
                                                      qos)
            self.coordinate_frame_ = CoordinateFrame.NONE
        elif self.nav_mode_ == NavMode.WAYPOINT:
            if self.coordinate_frame_ == CoordinateFrame.LOCAL:
                self.desired_ = Pose()
                self.desired_pub_ = self.create_publisher(PoseStamped,
                                                          topic_namespace+"/internal/behavior_local_pose",
                                                          qos)
            elif self.coordiante_frame_ == CoordinateFrame.GLOBAL:
                self.desired_ = NavSatFix()
                self.desired_pub_ = self.create_publisher(NavSatFix,
                                                          topic_namespace+"/internal/behavior_global_pose",
                                                          qos)
            elif self.coordinate_frame_ == CoordinateFrame.UTM:
                self.desired_ = UtmPose()
                self.desired_pub_ = self.create_publisher(NavSatFix,
                                                          topic_namespace+"/internal/behavior_global_pose",
                                                          qos)
            else:
                self.get_logger().info("Incorrect Coordinate Frame set, the options are LOCAL, GLOBAL, and UTM")
        else:
            self.get_logger().info("Incorrect Navigation Mode set, the options are VELOCITY and WAYPOINT")

        self.local_pose_ = LocalPose()
        self.global_pose_ = GlobalPose()

        self.system_ = dict()
        self.wp_distance_ = 0.0
        self.zone_ = None
        
    def localCallback(self, msg):
        self.local_pose_.initializeFromPoint(msg.position)

        
    def globalCallback(self, msg):
        self.global_pose_.initializeFromNavSatFix(msg)

        
    def systemArrayCallback(self, msg):
        for ag in msg.agents:
            self.system_[ag.sys_id] = ag

    def calcDistance(self):
        if self.coordinate_frame_ == CoordinateFrame.LOCAL:
            my_pose = (self.local_pose_.x, self.local_pose_.y)
            wp_pose = (self.desired_.position.x, self.desired_.position.y) 
            return math.dist(my_pose, wp_pose)
        elif self.coordinate_frame_ == CoordinateFrame.UTM:
            self.zone_, e, n = LLtoUTM(23, self.global_pose_.latitude, self.global_pose_.longitude)
            my_pose = (e,n)
            wp_pose = (self.desired_.easting, self.desired_.northing)
            return math.dist(my_pose, wp_pose)
        elif self.coordinate_frame_ == CoordinateFrame.GLOBAL:
            z, my_e, my_n = LLtoUTM(23, self.global_pose_.latitude, self.global_pose_.longitude)
            z, d_e, d_n = LLtoUTM(23, self.desired_.latitude, self.desired_.longitude)
            return math.dist(my_pose, wp_pose)

    def publishDesired(self, msg):
        #msg.header.stamp = self.get_clock().now()
        msg.header.frame_id = "behavior"
        self.desired_pub_.publish(msg)
        
    def cycleCallback(self):
        self.get_logger().info("publishing...")
        if self.nav_mode_ == NavMode.VELOCITY:
            msg = TwistStamped()
            msg.twist = self.desired_
            self.publishDesired(msg)
        elif self.nav_mode_ == NavMode.WAYPOINT:
            self.wp_distance_ = self.calcDistance()
            if self.coordinate_frame_ == CoordinateFrame.LOCAL:
                msg = PoseStamped()
                msg.pose = self.desired_
                self.publishDesired(msg)
            elif self.coordinate_frame_ == CoordinateFrame.GLOBAL:
                self.publishDesired(self.desired_)
            elif self.coordinate_frame_ == CoordinateFrame.UTM:
                lat, lon = UTMtoLL(23, self.desired_.northing, self.desired_.easting, self.zone_)
                msg = NavSatFix()
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = self.desired_.altitude
                self.publishDesired(msg)
        
        
