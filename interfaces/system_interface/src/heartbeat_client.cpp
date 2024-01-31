/* Author: Jason Hughes
 * Date: April 2023
 * About: Sender (client) node for heartbeat
 */

#include "system_interface/heartbeat_client.hpp"

HeartbeatClient::HeartbeatClient() : Node("heartbeat_client")
{
  const char* group_ = "226.1.1.1";

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error connecting to socket");
    }

  memset((char*) &address_, 0, sizeof(address_));

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = inet_addr(group_);
  address_.sin_port = htons(5005);

  local_interface_.s_addr = inet_addr("203.106.93.94");
  setsockopt(socket_fd_, IPPROTO_IP, IP_MULTICAST_IF, (char*) &local_interface_, sizeof(local_interface_));

  timer_ = this -> create_wall_timer(10000ms, std::bind(&HeartbeatClient::sendHeartbeat, this));
}
  

void HeartbeatClient::sendHeartbeat()
{
  const char* msg = "142";
  int nbytes = sendto(socket_fd_, msg, strlen(msg), 0, (struct sockaddr*) &address_, sizeof(address_));
  RCLCPP_INFO(this->get_logger(), "sending heartbeat");
  
  if (nbytes < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending heartbeat");
    }
}
