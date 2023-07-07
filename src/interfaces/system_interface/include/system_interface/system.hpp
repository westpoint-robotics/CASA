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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "casa_msgs/msg/casa_interface.hpp"
#include "casa_msgs/msg/casa_pose_array.hpp"
#include "casa_msgs/msg/casa_poses.hpp"
#include "casa_msgs/msg/casa_global_pose.hpp"
#include "casa_msgs/msg/casa_task.hpp"
#include "casa_msgs/msg/casa_task_array.hpp"
#include "casa_msgs/msg/casa_agent.hpp"
#include "casa_msgs/msg/casa_agent_array.hpp"
#include "system_interface/agent_tracker.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface();

private:

  std::vector<rclcpp::Subscription<casa_msgs::msg::CasaInterface>::SharedPtr> casa_references_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr external_sysid_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_sub_;
  
  rclcpp::Publisher<casa_msgs::msg::CasaAgentArray>::SharedPtr system_pose_pub_;
  rclcpp::Publisher<casa_msgs::msg::CasaInterface>::SharedPtr external_pub_;
  rclcpp::Publisher<casa_msgs::msg::CasaTaskArray>::SharedPtr system_task_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  void externalCasaCallback(const casa_msgs::msg::CasaInterface& msg);
  void localCallback(const geometry_msgs::msg::PoseStamped& msg);
  void internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg);
  void externalSysidCallback(const std_msgs::msg::UInt16MultiArray& msg);
  void headingCallback(const std_msgs::msg::Float32& msg);
  void myTaskCallback(const std_msgs::msg::Int32& msg);
  
  void timerCallback();

  void posePublisher();
  void exchangePublisher();
  void taskPublisher();
  
  void checkTime();
  
  std::map<int,int> iter_tracker_; 
  std::vector<AgentTracker> system_tracker_;
  std::vector<int> system_ids_;
  
  int my_id_;
  int my_task_;
  float dropout_;
  std::string namespace_;

  bool use_sim_;
  
  int sys_id_in_;
  int iterator_;
  int num_agents_;
  int task_in_;
  int level_in_;
  float heading_in_;
  float lat_in_, lon_in_, alt_in_;
  float internal_lat_, internal_lon_, internal_alt_, internal_heading_;
  float local_x_, local_y_;
  int conn_level_;
};
#endif
