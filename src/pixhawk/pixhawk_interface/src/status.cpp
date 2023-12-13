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

#include "px4_msgs/msg/failsafe_flags.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

// TODO: Add in debbuggigng statements and GCS interfacing. 

class StatusInterface : public rclcpp::Node
{
public:
  StatusInterface() : Node("status_interface")
  {
    // set the Quality of Service
    // RELIABILITY: best effort
    // DURABILITY: transient local
    // everything else is kept default
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.transient_local();

    this -> declare_parameter("sys_id", 1);

    sys_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
    
    std::string sys_namespace = "px4_"+ std::to_string(sys_id_);


    status_sub_ = this -> create_subscription<px4_msgs::msg::VehicleStatus>(sys_namespace+"/fmu/out/vehicle_status",
									    qos,
									    std::bind(&StatusInterface::status_callback, this, std::placeholders::_1));

    failsafe_sub_ = this -> create_subscription<px4_msgs::msg::FailsafeFlags>(sys_namespace+"/fmu/out/failsafe_flags",
									      qos,
									      std::bind(&StatusInterface::failsafe_callback, this, std::placeholders::_1));

  }

private:

  int sys_id_;

  int arming_state_, latest_disarming_reason_, nav_state_, system_id_;

  bool battery_low_, battery_unhealthy_;
  
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;

  void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void failsafe_callback(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);

};

void StatusInterface::status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  arming_state_ = msg->arming_state;
  latest_disarming_reason_ = msg->latest_disarming_reason;
  nav_state_ = msg->nav_state;
  system_id_ = msg->system_id;
}

void StatusInterface::failsafe_callback(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
  battery_low_ = msg->battery_low_remaining_time;
  battery_unhealthy_ = msg->battery_unhealthy;
}

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusInterface>());
  rclcpp::shutdown();
  
  return 0;
}
