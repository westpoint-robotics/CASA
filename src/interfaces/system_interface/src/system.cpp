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
#include <map>
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
	
	if (i != sys_id_in_)
	  {
	    auto gps_sub = this -> create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, qos, std::bind(&SystemInterface::gps_callback ,this, std::placeholders::_1));
          
            gps_references.push_back(gps_sub);
           }
       }
  }

private:

  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_references;

  void gps_callback(const sensor_msgs::msg::NavSatFix& msg);

  std::map<int,float(*)[3]> swarm_tracker_; 
  
  int sys_id_in_;
  float lat_in_, lon_in_, alt_in_;
  
};


void SystemInterface::gps_callback(const sensor_msgs::msg::NavSatFix& msg)
{
  sys_id_in_ = std::stoi(msg.header.frame_id);
  lat_in_ = msg.latitude;
  lon_in_ = msg.longitude;
  alt_in_ = msg.altitude;
  float data_in[3] = {lat_in_, lon_in_, alt_in_};
  
  swarm_tracker_.insert(std::pair<int,float(*)[3]>(sys_id_in_, &data_in));
  
  RCLCPP_INFO_STREAM(this->get_logger(),"swarm_tracker size: " << swarm_tracker_.size()) 
}


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemInterface>());
  rclcpp::shutdown();
  
  return 0;
}
