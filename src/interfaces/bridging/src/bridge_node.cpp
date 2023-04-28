/* Author: Jason Hughes
 * Date: April 2023
 * About: Main node
 */

#include <rclcpp/rclcpp.hpp>

#include "bridging/bridge.hpp"

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
  rclcpp::shutdown();

  return 0;
}
