/* Author: Jason Hughes
 * Date: April 2023
 * About: Headers for system node
 */

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdint.h>
#include <map>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "system_interface/agent_tracker.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface();

private:

  std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_references;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr internal_gps_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr external_sysid_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr system_pose_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  void externalGpsCallback(const sensor_msgs::msg::NavSatFix& msg);
  void localCallback(const geometry_msgs::msg::PoseStamped& msg);
  void internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg);
  void externalSysidCallback(const std_msgs::msg::UInt16MultiArray& msg);

  void timerCallback();

  void posePublisher();

  void checkTime();
  
  std::map<int,AgentTracker> system_tracker_; 

  std::vector<int> system_ids_;

  int my_id_;
  float dropout_;
  std::string namespace_;
  
  int sys_id_in_;
  float lat_in_, lon_in_, alt_in_;
  float internal_lat_, internal_lon_;
  float local_x_, local_y_;
};
#endif
