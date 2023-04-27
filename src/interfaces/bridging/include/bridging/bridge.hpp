/* Author: Jason Hughes
 * Date: April 2023
 * About: Headers for bridge node
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <domain_bridge/domain_bridge>

class Bridging
{
public:

  Bridging()

  
private:

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sys_id_sub_;

  void sysIdCallback(const std_msgs::msg::Int16MultiArray& msg);
  
  
}
