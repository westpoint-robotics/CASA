/* Author: Jason Hughes
 * Date: April 2022
 * About: Server side heartbeat caster headers 
 */

#ifndef HEARTBEAT_SERVER_HPP
#define HEARTBEAT_SERVER_HPP

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

#define PORT 5005
#define MSGBUFSIZE 1024

class HeartbeatServer : public rclcpp::Node
{
public:
  HeartbeatServer();

private:

  int server_fd_;
  int new_socket_;
  int valread_;
  int sys_id_in_;
  int my_id_;
  struct sockaddr_in address_;
  struct ip_mreq mreq_;
  
  int opt_;
  socklen_t addrlen_;
  char* msg_;

  std::vector<int> system_ids_;
  
  void recieveFrom(char buffer[]);
  void idPublisher();
  
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr system_id_pub_;

};
#endif
