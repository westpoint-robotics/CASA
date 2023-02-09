/*!
 * Author: Jason Hughes
 * Date:   Jan. 2023
 * About:  ROS2 node to interface with the rest of the swarm. 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
//#include "system_interface/system.hpp"

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface() : Node("system_interface")
  {
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.transient_local();

    this -> declare_parameter("sys_id", 1);
     
    //need the number of agents in the swarm 
    for (int i = 1; i <= 2; i++)
      {
	
	std::string status_topic = "px4_"+ std::to_string(i) + "/fmu/out/vehicle_status";
	std::string gps_topic = "px4_"+ std::to_string(i) + "/fmu/out/vehicle_gps_position";
	
	if (i != my_sys_id_)
	  {
	    auto status_sub = this -> create_subscription<px4_msgs::msg::VehicleStatus>(status_topic, qos, std::bind(&SystemInterface::status_callback ,this, std::placeholders::_1));
	    auto gps_sub = this -> create_subscription<px4_msgs::msg::SensorGps>(gps_topic, qos, std::bind(&SystemInterface::gps_callback ,this, std::placeholders::_1));
           
	    status_references.push_back(status_sub);
            gps_references.push_back(gps_sub);
           }
       }
  }

private:

  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> status_references;
  std::vector<rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr> gps_references;

  void status_callback(const px4_msgs::msg::VehicleStatus& msg);
  void gps_callback(const px4_msgs::msg::SensorGps& msg);

  int sys_id_in_;
  int my_sys_id_;
  
};

void SystemInterface::status_callback(const px4_msgs::msg::VehicleStatus& msg)
{
  sys_id_in_ = msg.system_id;
  RCLCPP_INFO_STREAM(this->get_logger(), "message recieved from: "<< sys_id_in_);
}

void SystemInterface::gps_callback(const px4_msgs::msg::SensorGps& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "recieved gps message");
}


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemInterface>());
  rclcpp::shutdown();
  
  return 0;
}
