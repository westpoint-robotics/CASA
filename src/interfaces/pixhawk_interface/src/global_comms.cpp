/*
 * Jason Hughes
 * Date: Feb. 2023
 * Description: Node to interface with the pixhawk
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class PixhawkInterface : public rclcpp::Node
{
public:
  PixhawkInterface() : Node("pixhawk_interface")
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
										std::bind(&PixhawkInterface::status_callback, this, std::placeholders::_1));
    gps_sub_ = this -> create_subscription<px4_msgs::msg::SensorGps>(sys_namespace+"/fmu/out/vehicle_gps_position",
									 qos,
									 std::bind(&PixhawkInterface::gps_callback, this, std::placeholders::_1));

    internal_gps_pub_ = this -> create_publisher<sensor_msgs::msg::NavSatFix>("/internal/global_position", qos);
    external_gps_pub_ = this -> create_publisher<sensor_msgs::msg::NavSatFix>("/external/global_position", qos);

    timer_ = this -> create_wall_timer(1000ms, std::bind(&PixhawkInterface::timer_callback, this));

  }

private:

  int sys_id_;

  float lat_in_, lon_in_, alt_in_;
  
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr internal_gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr external_gps_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void status_callback(const px4_msgs::msg::VehicleStatus& msg);
  void gps_callback(const px4_msgs::msg::SensorGps& msg);

  void internal_publisher(float lat, float lon, float alt);
  void external_publisher(float lat, float lon, float alt, int sys_id);

  float int_to_float_conversion(int input, int flag);
  
  void timer_callback();
};

float PixhawkInterface::int_to_float_conversion(int input, int flag)
{
  // utility function to convert incoming ints to outgoing floats 
  float output; 
  if (flag == 0) //altitude
    {
      output = input * 0.001;
    }
  else //lat or longitude
    {
      output = input * 0.0000001;
    }
  return output;
}

void PixhawkInterface::timer_callback()
{
  internal_publisher(lat_in_, lon_in_, alt_in_);
  external_publisher(lat_in_, lon_in_, alt_in_, sys_id_);
}

void PixhawkInterface::status_callback(const px4_msgs::msg::VehicleStatus& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "status message recieved at: "<< sys_id_);
}

void PixhawkInterface::gps_callback(const px4_msgs::msg::SensorGps& msg)
{
  lat_in_ = int_to_float_conversion(msg.lat, 1);
  lon_in_ = int_to_float_conversion(msg.lon, 2);
  alt_in_ = int_to_float_conversion(msg.alt, 0);
  RCLCPP_INFO_STREAM(this->get_logger(), "recieved gps message at: "<< sys_id_);
}

void PixhawkInterface::internal_publisher(float lat, float lon, float alt)
{
  sensor_msgs::msg::NavSatFix msg{};

  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;

  internal_gps_pub_->publish(msg);
}

void PixhawkInterface::external_publisher(float lat, float lon, float alt, int sys_id)
{
  sensor_msgs::msg::NavSatFix msg{};

  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.header.frame_id = sys_id;

  external_gps_pub_->publish(msg);
}


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixhawkInterface>());
  rclcpp::shutdown();
  
  return 0;
}


  
