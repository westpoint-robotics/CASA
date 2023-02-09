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
#include "px4_msgs/msg/vehicle_local_position.hpp"

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

    std::string sys_namespace = "px4_"+ std::to_string(i);
    
    
    status_sub_ = this -> create_subscription<px4_msgs::msg::VehicleStatus>(sys_namespace+"/fmu/out/vehicle_status",
										qos,
										std::bind(&PixhawkInterface::status_callback, this, std::placeholders::_1));
    gps_sub_ = this -> create_subscription<px4_msgs::msg::SensorGps>(sys_namespace+"/fmu/out/vehicle_gps_position",
									 qos,
									 std::bind(&PixhawkInterface::gps_callback ,this, std::placeholders::_1));

    pose_sub_ = this -> create_subscription<px4_msgs::msg::VehicleLocalPosition>(sys_namespace+"/fmu/out/vehicle_local_position",
										     qos,
										     std::bind(&PixhawkInterface::pose_callback ,this, std::placeholders::_1));

  }

private:

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPrt pose_sub_;

  void status_callback(const px4_msgs::msg::VehicleStatus& msg);
  void gps_callback(const px4_msgs::msg::SensorGps& msg);
  void pose_callback(const px4_msgs::msg::VehcileLocalPosition& msg);

};

void PixhawkInterface::status_callback(const px4_msgs::msg::VehicleStatus& msg)
{
  sys_id_in_ = msg.system_id;
  RCLCPP_INFO_STREAM(this->get_logger(), "message recieved from: "<< sys_id_in_);
}

void PixhawkInerface::gps_callback(const px4_msgs::msg::SensorGps& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "recieved gps message");
}

void PixhawkInterface::pose_callback(const px4_msgs::msg::VehicleLocalPosition& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "recieved gps message");
}




int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixhawkInterface>());
  rclcpp::shutdown();
  
  return 0;
}


  
