/* Author: Jason Hughes
 * Date: April 2022\
 * About: Server side heartbeat caster 
 */

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

#define PORT 5005
#define MSGBUFSIZE 1024

class HeartbeatServer : public rclcpp::Node
{
public:
  HeartbeatServer() : Node("heartbeat_server")
  {
    opt_ = 1;
    addrlen_ = sizeof(address_);
    char buffer[1024] = { 0 };

    const char* group = "239.255.255.250";
    //TODO: get group from a parameter
    server_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    
    if (server_fd_ < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Could not create socket file descriptor");
	exit(EXIT_FAILURE);
      }

    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt_, sizeof(opt_)) < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to set socket options");
	exit(EXIT_FAILURE);
      }

    memset((char*) &address_, 0, sizeof(address_));
    
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(PORT);

    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)))
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to bind to port");
	exit(EXIT_FAILURE);
      }

    mreq_.imr_multiaddr.s_addr = inet_addr("226.1.1.1");
    mreq_.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(server_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq_, sizeof(mreq_)) < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to join multicast group");
	exit(EXIT_FAILURE);
      }
    addrlen_ = sizeof(address_);
    
    recieveFrom(buffer);
  }
    
private:

  int server_fd_;
  int new_socket_;
  int valread_;
  struct sockaddr_in address_;
  struct ip_mreq mreq_;
  
  int opt_;
  socklen_t addrlen_;
  char* msg_;

  void recieveFrom(char buffer[]);
};

void HeartbeatServer::recieveFrom(char buffer[])
{
  
  while (true)
    {
      RCLCPP_INFO(this->get_logger(), "waiting...");
      addrlen_ = sizeof(address_);
      int nbytes = read(server_fd_, buffer, sizeof(buffer));
      if (nbytes < 0)
	{
	  RCLCPP_ERROR(this->get_logger(), "Error recieving message");
	  exit(EXIT_FAILURE);
	}
      buffer[nbytes] = '\0';
      
      RCLCPP_INFO_STREAM(this->get_logger(), "got heartbeat from "<<buffer);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<HeartbeatServer>() );

  rclcpp::shutdown();

  return 0;
}
