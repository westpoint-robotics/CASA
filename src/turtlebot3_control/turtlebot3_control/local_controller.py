#!/usr/bin/env python

# Jordan Beason
# Dec 15 23
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription
        self.target_x = 3.0  #Target X
        self.target_y = 2.0  #Target Y
        self.reached = False

    def odom_callback(self, msg):
        if self.reached:
            return
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        yaw = self.get_yaw(orientation_q)

        #Distance Calcs
        dx = self.target_x - position.x
        dy = self.target_y - position.y
        distance = math.sqrt(dx**2 + dy**2)

        #Angle Calcs
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - yaw)

        #Speed Control
        linear_speed = 0.2
        angular_speed = 0.5

        twist = Twist()
        if distance < 0.1:
            self.reached = True
            self.get_logger().info('Target reached')
        else:
            #Linear velocity
            twist.linear.x = linear_speed if abs(angle_diff) < math.pi / 6 else 0.0

            #Angular velocity
            twist.angular.z = angular_speed * angle_diff

        #Publish
        self.publisher_.publish(twist)

    def get_yaw(self, quat): #Convert the quaternion from /odom to yaw
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        sin_y = 2.0 * (w * z + x * y)
        cos_y = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_y, cos_y)
        return yaw

    def normalize_angle(self, angle):     #Normalize the angle to the range [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    simple_navigator = SimpleNavigator()
    rclpy.spin(simple_navigator)
    simple_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
