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

    //declare parameters below

    //rclcpp::executors::SingleThreadExecutor exec;
    //std::vector<rclcpp::Node::SharedPtr> node_references;
    //std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> sub_references;

    for (int i = 0; i < 2; i++)
      {
	std::string topic = "px4_"+ std::to_string(i) + "/fmu/out/vehicle_status";
	auto sub = this -> create_subscription<px4_msgs::msg::VehicleStatus>(topic,
									     qos,
									     std::bind(&SystemInterface::status_callback,
										       this,
										       std::placeholders::_1));

	sub_references.push_back(sub);
	
      }
  }

private:

  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> sub_references;

  void status_callback(const px4_msgs::msg::VehicleStatus& msg);

};

void SystemInterface::status_callback(const px4_msgs::msg::VehicleStatus& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "message recieved");
}

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemInterface>());
  rclcpp::shutdown();
  
  return 0;
}
