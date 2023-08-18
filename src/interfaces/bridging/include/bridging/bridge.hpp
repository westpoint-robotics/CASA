/* Author: Jason Hughes
 * Date: April 2023
 * About: Headers for bridge node
 */

#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <string>
#include <chrono>

#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/topic_bridge_options.hpp"
#include "utils/casa_utils.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class Bridge : public rclcpp::Node
{
public:

  Bridge();

private:

  int my_id_;
  std::vector<long int> system_ids_;
  std::set<std::string> bridged_topics_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::SubscriptionOptions options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sys_id_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  domain_bridge::DomainBridge domain_bridge_;

  std::map<std::string, std::vector<std::string>> topic_map_;

  void timerCallback();
  void sysIdCallback(const std_msgs::msg::UInt16MultiArray& msg);
  void extractTopicsAndBridge();
  void addBridge(std::string topic, std::string type);
};
#endif
