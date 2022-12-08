/*
 * Author: Jason Hughes
 * Date: December 2022
 * Description: A simple script to take off a single uav
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

using namespace std::chrono;

class TakeoffControl : public rclcpp::Node
{
public:
  TakeoffControl() : Node("takeoff_node")
  {
    
    takeoff_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    timer_ = this -> create_wall_timer(5ms, std::bind(&TakeoffControl::takeoff_pub_callback, this));
  }
private:
  void takeoff_pub_callback()
  {
    auto msg = px4_msgs::msg::VehicleStatus();
    msg.nav_state = 17;
    takeoff_pub_->publish(msg);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr takeoff_pub_;
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffControl>());
  rclcpp::shutdown();
  
  return 0;
}
