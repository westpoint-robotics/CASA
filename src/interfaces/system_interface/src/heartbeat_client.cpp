/* Author: Jason Hughes
 * Date: April 2023
 * About: Sender (client) node for heartbeat
 */

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
  HeartbeatClient() : Node("heartbeat_client")
  {
    const char* group_ = "239.255.25.250";

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_fd_ < 0)
      {
	RCLCPP_ERROR(this->get_logger(), "Error connecting to socket");
      }

    memset(&address_, 0, sizeof(address_));

    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = inet_addr(group_);
    address_.sin_port = htons(5005);

    timer_ = this -> create_wall_timer(10000ms, std::bind(&HeartbeatClient::sendHeartbeat, this));
  }
  
private:
  std::string group_;
  int socket_fd_;

  struct sockaddr_in address_;

  rclcpp::TimerBase::SharedPtr timer_;

  void sendHeartbeat();
};

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


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatClient>());

  rclcpp::shutdown();
  
  return 0;
}
