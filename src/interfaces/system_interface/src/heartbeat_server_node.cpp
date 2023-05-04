/* Author: Jason Hughes
 * Date: APril 2023
 * About: node for heartbeat server
 */

#include "system_interface/heartbeat_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<HeartbeatServer>() );

  rclcpp::shutdown();

  return 0;
}
