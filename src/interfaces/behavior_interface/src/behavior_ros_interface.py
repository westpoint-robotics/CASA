"""
Author: Jason Hughes
Date: July 2023
About: Behavior organization node
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from casa_msgs.msg import CasaAgent, CasaAgentArray
from geometry_msgs import PoseStamped, TwistStamped

from behavior_interface.nav_mode import NavMode
from behavior_interface.coordinate_frame import CoordinateFrame


class BehaviorRosInterface(Node):

    def __init__(self, nav_mode, coordinate_frame):

        super().__init__("BehaviorRosInterface")

        qos = QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT,
                         durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10 )

        self.nav_mode_ = nav_mode
        self.coordinate_frame_ = coordinate_frame

        self.declare_parameter("sys_id". rclpy.Parameter.Type.INTEGER)
        self.sys_id_ = self.get_parameter("sys_id").value

        topic_namespace = "casa"+str(self.sys_id_)

        # subscriptions
        self.local_pose_sub_ = self.create_subscription(PoseStamped, topic_namespace+"/internal/local_position",
                                                        self.localCallback, qos) 
        self.global_pose_sub_ = self.create_subscription(NavSatFix, topic_namespace+"/internal/global_position",
                                                         self.globalCallback, qos)
        self.system_poses_ = self.create_subscription(CasaAgentArray, topic_namespace+"/internal/system_array",
                                                      self.poseArrayCallback,
                                                      qos)

        # publishers
        self.vel_pub_ = self.create_publisher(TwistStamped,
                                              topic_namespace+"/internal/behavior_vel",
                                              qos)
        self.local_pose_pub_ = self.create_publsiher(PoseStamped,
                                                     topic_namespace"/internal/behavior_local_pose",
                                                     qos)
        self.global_pose_pub_ = self.create_publisher(NavSatFix,
                                                      topic_namespace"/internal/behavior_global_pose",
                                                      qos)

        # timer
        self.timer_ = self.create_timer(0.1, self.cycleCallback)


                                                    
