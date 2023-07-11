/* Author: Jason Hughes
 * Date: July 2023
 * About: trajectory planning for behaviors
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;


class TrajectoryPlanner : public rclcpp::Node
{
public:
  TrajectoryPlanner();

private:

  void poseCallback(const geometry_msgs::msg::PoseStamped& msg);
  void velCallback(const geometry_msgs::msg::TwistStamped& msg);
  void headingCallback(const std_msgs::msg::Float32 & msg);
  
  void toPixhawk();

  void publishControlMode();
  void publishTrajectory();
  void publishVehicleCommand(uint16_t command, float param1, float param2);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

  int my_id_;
  float default_alt_;
  float heading_;
  float pose_x_, pose_y_, pose_z_;
  float vel_x_, vel_y_, vel_z_;

  bool use_pose_, use_vel_;
};

TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner")
{
  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.best_effort();
  qos.transient_local();

  this -> declare_parameter("sys_id", 1);
  this -> declare_parameter("default_alt", 10.0);
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  default_alt_ = this -> get_parameter("default_alt").get_parameter_value().get<float>();
  
  std::string pub_namespace = "/px4_" + std::to_string(my_id_);
  std::string sub_namespace = "/casa" + std::to_string(my_id_);

  control_mode_pub_ = this -> create_publisher<px4_msgs::msg::OffboardControlMode>(pub_namespace+"/fmu/in/offboard_control_mode", qos);
  vehicle_command_pub_ = this -> create_publisher<px4_msgs::msg::VehicleCommand>(pub_namespace+"/fmu/in/vehicle_command", qos);
  trajectory_pub_ = this -> create_publisher<px4_msgs::msg::TrajectorySetpoint>(pub_namespace+"/fmu/in/trajectory_setpoint", qos);

  pose_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>(sub_namespace+"/internal/goto_pose",
									   qos,
									   std::bind(&TrajectoryPlanner::poseCallback,
										     this,
										     std::placeholders::_1));
  vel_sub_ = this -> create_subscription<geometry_msgs::msg::TwistStamped>(sub_namespace+"/internal/goto_vel",
									   qos,
									   std::bind(&TrajectoryPlanner::velCallback,
										     this,
										     std::placeholders::_1));

  heading_sub_ = this -> create_subscription<std_msgs::msg::Float32>(sub_namespace+"/internal/goto_heading",
								     qos,
								     std::bind(&TrajectoryPlanner::headingCallback,
									       this,
									       std::placeholders::_1));
  
  timer_ = this -> create_wall_timer(10ms, std::bind(&TrajectoryPlanner::toPixhawk, this));

  for(int i=0; i<5; i++)
    {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

  use_pose_ = true;
  use_vel_ = false;

  pose_x_ = 0.0;
  pose_y_ = 0.0;
  pose_z_ = -1.0 * default_alt_;
}

void TrajectoryPlanner::headingCallback(const std_msgs::msg::Float32& msg)
{
  heading_ = msg.data;
}

void TrajectoryPlanner::poseCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  pose_x_ = msg.pose.position.x;
  pose_y_ = msg.pose.position.y;
  pose_z_ = -1.0 * msg.pose.position.z;

  use_pose_ = true;
  use_vel_ = false;
}

void TrajectoryPlanner::velCallback(const geometry_msgs::msg::TwistStamped& msg)
{
  vel_x_ = msg.twist.linear.x;
  vel_y_ = msg.twist.linear.y;
  vel_z_ = msg.twist.linear.z;

  use_vel_ = true;
  use_pose_ = false;
}


void TrajectoryPlanner::publishControlMode()
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;
  msg.position = use_pose_;
  msg.velocity = use_vel_;
  msg.acceleration = false;

  control_mode_pub_ -> publish(msg);
}

void TrajectoryPlanner::publishTrajectory()
{
  px4_msgs::msg::TrajectorySetpoint msg;
  if (use_pose_)
    {
      msg.position = {pose_x_, pose_y_, pose_z_};
      msg.velocity = {};
    }
  else if (use_vel_)
    {
      msg.velocity = {vel_x_, vel_y_, vel_z_};
      msg.position = {};
    }
  msg.yaw = heading_;
  msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;

  trajectory_pub_ -> publish(msg);
}

void TrajectoryPlanner::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this -> get_clock()->now().nanoseconds()/1000;

  vehicle_command_pub_ -> publish(msg);
}

void TrajectoryPlanner::toPixhawk()
{
  publishControlMode();
  publishTrajectory();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanner>());
  rclcpp::shutdown();

  return 0;
}
