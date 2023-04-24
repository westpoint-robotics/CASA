/* Author: Jason Hughes
 * Date: April 2022\
 * About: Server side heartbeat caster 
 */

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

#define PORT 8088
#define MSGBUFSIZE 1024

class HeartbeatServer : public rclcpp::Node
{
public:
  HeartbeatServer() : Node("heartbeat_server")
  {
    opt_ = 1;
    addrlen_ = sizeof(address_);
    buffer_[1024] = { 0 };

    char* group = "239.255.255.250";
    //TODO: get group from a parameter
    
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Could not create socket file descriptor");
	exit(EXIT_FAILURE);
      }

    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_)))
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to set socket options");
	exit(EXIT_FAILURE);
      }

    memset(&address_, 0, sizeof(address_));
    
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(PORT);

    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to bind to port");
	exit(EXIT_FAILURE);
      }

    mreq_.imr_multiaddr.s_addr = inet_addr(group);
    mreq_.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(server_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq_, sizeof(mreq_)) < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Unable to join multicast group");
	exit(EXIT_FAILURE);
      }

    recieveFrom();
  }
    
private:

  int server_fd_;
  int new_socket_;
  int valread_;

  struct sockaddr_in address_;
  struct ip_mreq mreq_;
  
  int opt_;
  socklen_t addrlen_;
  char buffer_[];
  char* msg_;

  void recieveFrom();
};

void HeartbeatServer::recieveFrom()
{
  
  while (true)
    {
      addrlen_ = sizeof(address_);
      int nbytes = recvfrom(server_fd_, buffer_, MSGBUFSIZE, 0, (struct sockaddr *) &address_, &addrlen_);
      if (nbytes < 0)
	{
	  RCLCPP_ERROR(this->get_logger(), "Error recieving message");
	  exit(EXIT_FAILURE);
	}
      buffer_[nbytes] = '\0';
      puts(buffer_);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<HeartbeatServer>() );

  rclcpp::shutdown();

  return 0;
}
