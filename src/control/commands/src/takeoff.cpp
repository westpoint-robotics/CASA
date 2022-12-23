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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;


class TakeoffControl : public rclcpp::Node
{
public:
  TakeoffControl() : Node("takeoff_node")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    
    takeoff_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 15.0);
    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
								      std::bind(&TakeoffControl::gps_callback, this,
										std::placeholders::_1));
    setpoint_counter_ = 0;
    
    auto timer_callback = [this]() -> void
    {
      //param7 is altitude
      if (setpoint_counter_ <= 10)
      {
	RCLCPP_INFO(this->get_logger(), "Sending takeoff command");
	this -> publish_takeoff_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 10);
	this -> arm(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
	setpoint_counter_++;
      }
    };
    timer_ = this -> create_wall_timer(100ms, timer_callback);
  }

  void arm(uint16_t command);
  //void disarm();
  
private:

  void gps_callback(const px4_msgs::msg::SensorGps & msg) const
  {
    std::cout<< "in the callback";
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr takeoff_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sensor_sub_;
  
  std::atomic<uint64_t> timestamp_;
  int setpoint_counter_;
  
  void publish_takeoff_command(uint16_t command, float alt = 0.0);
};

void TakeoffControl::arm(uint16_t command)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.command = command;
  msg.param1 = 1.0;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
  takeoff_pub_->publish(msg);
}

void TakeoffControl::publish_takeoff_command(uint16_t command, float alt)
{
  
  px4_msgs::msg::VehicleCommand msg{};
  msg.command = command;
  msg.param5 = 41.390725;
  msg.param6 = -73.953215;
  msg.param7 = 498.8097;
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
