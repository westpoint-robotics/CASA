/* Author: Jason Hughes
 * Date: May 2023
 * About: Node for system object
 */

#include "system_interface/system.hpp"

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemInterface>());
  rclcpp::shutdown();

  return 0;
}
