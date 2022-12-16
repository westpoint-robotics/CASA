/*
 * Author: Jason Hughes
 * Date: December 2022
 * Description: A simple script to take off a single uav
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;

class TakeoffControl : public rclcpp::Node
{
public:
  TakeoffControl() : Node("takeoff_node")
  {
    
    takeoff_pub_ = this -> create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    //vehicle_status_sub_ = this
    
    auto timer_callback = [this]() -> void
    {
      //param7 is altitude
      this -> publish_takoff_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 10);
      this -> arm();
    }
    timer_ = this -> create_wall_timer(100ms, timer_callback);
  }

  void arm();
  void disarm();
  
private:
 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr takeoff_pub_;

  std::atomic<uint64_t> timestamp_;

  void publish_takeoff_command(uint16_t command, float alt = 0.0);
};


void TakeoffControl::publish_takoff_command(uint16_t command, float alt)
{
  px4_msgs::msg::VehcileCommand msg{};
  msg.command = command;
  msg.param7 = alt;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
  takeoff_pub_->publish(msg);
}


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffControl>());
  rclcpp::shutdown();
  
  return 0;
}
