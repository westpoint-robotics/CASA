/*
 * Jason Hughes
 * Date: Feb. 2023
 * Description: Node to interface with the pixhawk for global and local position. 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class TurtlebotInterface : public rclcpp::Node
{
public:
  TurtlebotInterface() : Node("turtlebot_interface")
  {
    rclcpp::QoS qos_turtlebot_odom(10);
    qos_turtlebot_odom.keep_last(10);
    qos_turtlebot_odom.best_effort();
    qos_turtlebot_odom.durability_volatile();

    rclcpp::QoS qos_vel(10);
    qos_vel.keep_last(10);
    qos_vel.reliable();
    qos_vel.transient_local();
    
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.transient_local();
    
    this -> declare_parameter("sys_id", 1);
    sys_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();

    std::string pub_nmspc = "casa"+ std::to_string(sys_id_);

    odom_sub_ = this -> create_subscription<nav_msgs::msg::Odometry>("/odom",
								     qos_turtlebot_odom,
								     std::bind(&TurtlebotInterface::odomCallback,
									       this,
									       std::placeholders::_1));
    vel_sub_ = this -> create_subscription<geometry_msgs::msg::TwistStamped>(pub_nmspc+"/internal/goto_vel",
									     qos,
									     std::bind(&TurtlebotInterface::velCallback,
										       this,
										       std::placeholders::_1));
    
    local_pub_ = this -> create_publisher<geometry_msgs::msg::PoseStamped>(pub_nmspc+"/internal/local_position",qos);
    vel_pub_ = this -> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",qos_vel);

  }

private:

  int sys_id_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void odomCallback(const nav_msgs::msg::Odometry& msg);
  void velCallback(const geometry_msgs::msg::TwistStamped& msg);

};

void TurtlebotInterface::odomCallback(const nav_msgs::msg::Odometry& msg)
{
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Connected to turtlebot3");
  geometry_msgs::msg::PoseStamped outgoing_msg;

  outgoing_msg.pose = msg.pose.pose;
  outgoing_msg.header.stamp = this -> get_clock() -> now();
  outgoing_msg.header.frame_id = "local";

  local_pub_ -> publish(outgoing_msg);
}

void TurtlebotInterface::velCallback(const geometry_msgs::msg::TwistStamped& msg)
{
  geometry_msgs::msg::Twist outgoing_msg;

  outgoing_msg = msg.twist;

  vel_pub_ -> publish(outgoing_msg);
}


int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotInterface>());
  rclcpp::shutdown();

  return 0;
}
