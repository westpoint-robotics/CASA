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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "system_interface/agent_tracker.hpp"

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

    local_pose_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>("/internal/local_position", //update this topic name
										     qos,
										     std::bind(&SystemInterface::localCallback, this, std::placeholders::_1));
    internal_gps_sub_ = this -> create_subscription<sensor_msgs::msg::NavSatFix>("/external/global_position", //update this topic name
										 qos,
										 std::bind(&SystemInterface::internalGpsCallback, this, std::placeholders::_1));
    //need the number of agents in the swarm 
    for (int i = 1; i <= 2; i++)
      {
	
	std::string gps_topic = "casa_"+ std::to_string(i) + "/external/global_position";
	
	if (i != sys_id_in_)
	  {
	    auto gps_sub = this -> create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic,
										    qos,
										    std::bind(&SystemInterface::externalGpsCallback ,this, std::placeholders::_1));
            gps_references.push_back(gps_sub);
           }
       }
  }

private:

  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_references;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr internal_gps_sub_;
  
  void externalGpsCallback(const sensor_msgs::msg::NavSatFix& msg);
  void localCallback(const geometry_msgs::msg::PoseStamped& msg);
  void internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg);

  std::map<int,AgentTracker> system_tracker_; 

  std::vector<int> system_ids_;
  std::vector<AgentTracker> agent_tracker_;
  
  int sys_id_in_;
  float lat_in_, lon_in_, alt_in_;
  float internal_lat_, internal_lon_;
  float local_x_, local_y_;
};


void SystemInterface::localCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  local_x_ = msg.pose.position.x;
  local_y_ = msg.pose.position.y;
}


void SystemInterface::internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  internal_lat_ = msg.latitude;
  internal_lon_ = msg.longitude;
}


void SystemInterface::externalGpsCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  sys_id_in_ = std::stoi(msg.header.frame_id);
  lat_in_ = msg.latitude;
  lon_in_ = msg.longitude;
  alt_in_ = msg.altitude;

  if (std::count(system_ids_.begin(), system_ids_.end(), sys_id_in_))
    {
      // if the sys_id is in the list, update the 
      system_tracker_[sys_id_in_].setLatLon(lat_in_, lon_in_);
      system_tracker_[sys_id_in_].setAlt(alt_in_);
      system_tracker_[sys_id_in_].calcRelativeXY(internal_lat_, internal_lon_, local_x_, local_y_);
    }
  else
    {
      AgentTracker agent = AgentTracker(sys_id_in_, lat_in_, lon_in_, alt_in_, internal_lat_, internal_lon_, local_x_, local_y_);
      system_ids_.push_back(sys_id_in_);
      system_tracker_.insert(std::pair<int,AgentTracker>(sys_id_in_,agent));
    }

}


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemInterface>());
  rclcpp::shutdown();
  
  return 0;
}
