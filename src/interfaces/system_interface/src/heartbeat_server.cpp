/* Author: Jason Hughes
 * Date: April 2022
 * About: Server side heartbeat caster 
 */

#include "system_interface/heartbeat_server.hpp"


HeartbeatServer::HeartbeatServer() : Node("heartbeat_server")
{
  // create and get the socket
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

  // ROS stuff
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  std::string nspc = "casa" + std::to_string(my_id_);

  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.best_effort();
  qos.transient_local();
    
  system_id_pub_ = this -> create_publisher<std_msgs::msg::UInt16MultiArray>(nspc+"/internal/system_ids", qos);
    
  recieveFrom(buffer);
}
    

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
      sys_id_in_ = std::stoi(buffer);

      if ((!(std::count(system_ids_.begin(), system_ids_.end(), sys_id_in_))) && (sys_id_in_ != my_id_))
	{
	  system_ids_.push_back(sys_id_in_);
	}
      
      
      RCLCPP_INFO_STREAM(this->get_logger(), "got heartbeat from "<<buffer);
    }
}


void HeartbeatServer::idPublisher()
{
  std_msgs::msg::UInt16MultiArray msg;
  msg.data.clear();
  msg.data.insert(msg.data.end(), system_ids_.begin(), system_ids_.end());
  msg.layout.data_offset = system_ids_.size();
  
  system_id_pub_ -> publish(msg);
}
