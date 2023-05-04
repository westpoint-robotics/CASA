/* Author: Jason Hughes
 * Date: April 2023
 * About: Sender (client) node for heartbeat
 */

#ifndef HEARTBEAT_CLIENT_HPP
#define HEARTBEAT_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;


class HeartbeatClient : public rclcpp::Node
{
public:
  
  HeartbeatClient();
  
private:
  std::string group_;
  int socket_fd_;

  struct sockaddr_in address_;
  struct in_addr local_interface_;

  rclcpp::TimerBase::SharedPtr timer_;

  void sendHeartbeat();
};
#endif
