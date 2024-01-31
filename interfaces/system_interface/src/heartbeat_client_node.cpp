/* Author: Jason Hughes
 * Date: April 2023
 * About: Sender (client) node for heartbeat 
 */

#include "system_interface/heartbeat_client.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatClient>());

  rclcpp::shutdown();
  
  return 0;
}
