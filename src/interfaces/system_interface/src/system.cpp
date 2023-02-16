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

#include "sensor_msgs/msg/nav_sat_fix.hpp"
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
	
	std::string gps_topic = "casa_"+ std::to_string(i) + "/external/global_position";
	
	if (i != my_sys_id_)
	  {
	    auto gps_sub = this -> create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, qos, std::bind(&SystemInterface::gps_callback ,this, std::placeholders::_1));
          
            gps_references.push_back(gps_sub);
           }
       }
  }

private:

  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_references;

  void gps_callback(const sensor_msgs::msg::NavSatFix& msg);

  int sys_id_in_;
  int my_sys_id_;
  
};


void SystemInterface::gps_callback(const sensor_msgs::msg::NavSatFix& msg)
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
