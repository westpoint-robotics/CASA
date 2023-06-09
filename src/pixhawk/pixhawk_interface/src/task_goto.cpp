/* Author: Jason Hughes
 * Date: June 2023
 * About: Node to take in task info and send it to the pixhawk
 */

#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class GoToTask : public rclcpp::Node
{
public:

  GoToTask();

private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr task_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  int my_id_;
  float task_x_in_, task_y_in_, alt_;

  px4_msgs::msg::VehicleStatus status_;
  
  void cycleCallback();
  void taskCallback(const geometry_msgs::msg::PoseStamped& msg);
  void statusCallback(const px4_msgs::msg::VehicleStatus& msg);
  void publishControlMode();
  void publishTrajectory();
};

GoToTask::GoToTask() : Node("goto_task")
{
  // set QoS
  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.best_effort();
  qos.transient_local();
  
  // get sys_id
  this -> declare_parameter("sys_id", 1);
  this -> declare_parameter("altitude", 10);
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  alt_ = this -> get_parameter("altitude").get_parameter_value().get<float>();
  
  std::string pub_namespace = "/px4_" + std::to_string(my_id_);
  std::string sub_namespace = "/casa_" + std::to_string(my_id_);
  
  control_mode_pub_ = this -> create_publisher<px4_msgs::msg::OffboardControlMode>(pub_namespace+"/fmu/in/offboard_control_mode", qos);
  vehicle_command_pub_ = this -> create_publisher<px4_msgs::msg::VehicleCommand>(pub_namespace+"/fmu/in/vehicle_command", qos);
  trajectory_pub_ = this -> create_publisher<px4_msgs::msg::TrajectorySetpoint>(pub_namespace+"/fmu/in/trajectory_setpoint", qos);

  task_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>(sub_namespace+"/internal/task",
									   qos,
									   std::bind(&GoToTask::taskCallback,
										     this,
										     std::placeholders::_1));

  status_sub_ = this -> create_subscription<px4_msgs::msg::VehicleStatus>(pub_namespace+"/fmu/out/vehicle_status",
									  qos,
									  std::bind(&GoToTask::statusCallback,
										    this,
										    std::placeholders::_1));
  
  timer_ = this -> create_wall_timer(10ms, std::bind(&GoToTask::cycleCallback, this));
  
}

void GoToTask::cycleCallback()
{
  publishControlMode();

  if (status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      //publish point from task
    }
}


void GoToTask::taskCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  task_x_in_ = msg.pose.position.x;
  task_y_in_ = msg.pose.position.y;
}


void GoToTask::statusCallback(const px4_msgs::msg::VehicleStatus& msg)
{
  status_ = msg;
}


void GoToTask::publishControlMode()
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;

  control_mode_pub_ -> publish(msg);
}


void GoToTask::publishTrajectory()
{
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.position = {task_x_in_, task_y_in_, alt_};
  msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;
  // TODO: calculate heading in radians

  trajectory_pub_ -> publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToTask>());
  rclcpp::shutdown();

  return 0;
}
