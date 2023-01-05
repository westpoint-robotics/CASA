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
    // set the Quality of Service
    // RELIABILITY: best effort
    // DURABILITY: transient local
    // everything else is kept default
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.transient_local();

    this -> declare_parameter("takeoff_alt", 10);
    
    takeoff_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
								      std::bind(&TakeoffControl::gps_callback, this,
										std::placeholders::_1));
    setpoint_counter_ = 0;
    
    auto timer_callback = [this]() -> void
    {
      //param7 is altitude
      takeoff_alt_param_in_ = this -> get_parameter("takeoff_alt").get_parameter_value().get<int>();
      
      if (setpoint_counter_ <= 10)
      {
	RCLCPP_INFO(this->get_logger(), "Sending takeoff command");
	this -> publish_takeoff_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
					alt_in_ + takeoff_alt_param_in_, lat_in_, lon_in_);
	this -> arm(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
	setpoint_counter_++;
      }
      if (setpoint_counter_ == 10)
      {
	RCLCPP_INFO(this->get_logger(), "Takeoff detected, shuting down node.");
	rclcpp::shutdown();
      }
    };
    timer_ = this -> create_wall_timer(100ms, timer_callback);
  }

  void arm(uint16_t command);
  //void disarm();
  
private:

  void gps_callback(const px4_msgs::msg::SensorGps& msg)
  {
    //RCLCPP_INFO_STREAM(this->get_logger(),"lat: "<< msg.lat);
    alt_in_ = int_to_float_conversion(msg.alt, 0);
    lat_in_ = int_to_float_conversion(msg.lat, 1);
    lon_in_ = int_to_float_conversion(msg.lon, 2);
    RCLCPP_INFO_STREAM(this->get_logger(),"Alt: "<< alt_in_);
  }

  float int_to_float_conversion(int input, int flag)
  {
    float output; 
    if (flag == 0) //altitude
      {
  	output = input * 0.001;
      }
    else //lat or longitude
      {
  	output = input * 0.0000001;
      }
    std::cout << output << std::endl;
    return output;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr takeoff_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sensor_sub_;
  
  std::atomic<uint64_t> timestamp_;
  int setpoint_counter_;
  int takeoff_alt_param_in_;
  float alt_in_ = 0;
  float lat_in_ = 0;
  float lon_in_ = 0;
  
  void publish_takeoff_command(uint16_t command, float alt, float lat, float lon);
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

void TakeoffControl::publish_takeoff_command(uint16_t command, float alt, float lat, float lon)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.command = command;
  msg.param5 = lat;
  msg.param6 = lon;
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
