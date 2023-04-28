/* Author: Jason Hughes
 * Date: April 2023
 * About: Headers for bridge node
 */

#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <string>

#include "domain_bridge/domain_bridge.hpp"
#include "utils/casa_utils.hpp"

class Bridge : public rclcpp::Node
{
public:

  Bridge();

private:

  int my_id_;
  std::vector<uint16_t> system_ids_;
  std::vector<std::string> bridge_topics_;
  
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sys_id_sub_;
  domain_bridge::DomainBridge domain_bridge_;

  std::map<std::string, std::vector<std::string>> topic_map_;
  
  void sysIdCallback(const std_msgs::msg::UInt16MultiArray& msg);
  void getTopics();
};
#endif
