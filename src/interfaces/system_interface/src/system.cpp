/*
 * Author: Jason Hughes
 * Date:   Jan. 2023
 * About:  ROS2 node to interface with the rest of the swarm. 
 */

#include "system_interface/system.hpp"


SystemInterface::SystemInterface() : Node("system_interface")
{
  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.best_effort();
  qos.transient_local();

  this -> declare_parameter("sys_id", 1);
  this -> declare_parameter("dropout", 30.0);
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  dropout_ = this -> get_parameter("dropout").get_parameter_value().get<float>();
    
  namespace_ = "casa" + std::to_string(my_id_);
    
  local_pose_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>(namespace_+"/internal/local_position", //update this topic name
										 qos,
										 std::bind(&SystemInterface::localCallback, this, std::placeholders::_1));
  global_pose_sub_ = this -> create_subscription<sensor_msgs::msg::NavSatFix>(namespace_+"/internal/global_position", //update this topic name
									       qos,
									       std::bind(&SystemInterface::internalGpsCallback, this, std::placeholders::_1));

  heading_sub_ = this -> create_subscription<std_msgs::msg::Float32>(namespace_+"/internal/heading",
								     qos,
								     std::bind(&SystemInterface::headingCallback, this, std::placeholders::_1));

  //need the number of agents in the swarm 
  for (int i = 1; i <= 2; i++)
    {
	
      std::string casa_topic = "casa"+std::to_string(i)+"/external/exchange";
	
      if (i != my_id_)
	{
	  auto casa_sub = this -> create_subscription<casa_msgs::msg::CasaInterface>(casa_topic,
										     qos,
										     std::bind(&SystemInterface::externalGpsCallback ,this, std::placeholders::_1));
	  casa_references_.push_back(casa_sub);
	}
    }

  system_pose_pub_ = this -> create_publisher<geometry_msgs::msg::PoseArray>(namespace_+"/internal/system_poses", qos);
  external_pub_ = this -> create_publisher<casa_msgs::msg::CasaInterface>(namespace_+"/external/exchange", qos);
  
  timer_ = this -> create_wall_timer(1000ms, std::bind(&SystemInterface::timerCallback, this));
}


void SystemInterface::timerCallback()
{
  posePublisher();

  if (dropout_ != 0)
    {
      checkTime();
    }
  
}


void SystemInterface::checkTime()
{
  // function to remove elements from the system_tracker if you have not heard from them
  std::map<int,AgentTracker>::iterator iter;
  rclcpp::Time now = this -> get_clock() -> now();
  
  for (iter = system_tracker_.begin(); iter != system_tracker_.end(); ++iter)
    {
      rclcpp::Time last_seen = iter->second.getTime();
      
      if ( now.seconds() - last_seen.seconds() > dropout_ )
	{
	  system_tracker_.erase(iter);
	}
    }
}


void SystemInterface::exchangePublisher()
{
  casa_msgs::msg::CasaInterface msg;

  msg.latitude = internal_lat_;
  msg.longitude = internal_lon_;
  msg.altitude = 
  
}


void SystemInterface::posePublisher()
{
  geometry_msgs::msg::PoseArray poses_arr_msg;
  std::map<int,AgentTracker>::iterator i;
  std::vector<geometry_msgs::msg::Pose> pose_vec;

  for(i = system_tracker_.begin(); i != system_tracker_.end(); ++i)
    {
      geometry_msgs::msg::Pose agent_reference_pose;

      agent_reference_pose.position.x = i->second.getRelativeXY()[0];
      agent_reference_pose.position.y = i->second.getRelativeXY()[1];
      agent_reference_pose.position.z = i->second.getAlt();

      pose_vec.push_back(agent_reference_pose);
    }

  poses_arr_msg.poses = pose_vec;
  poses_arr_msg.header.stamp = this -> get_clock() -> now();
  poses_arr_msg.header.frame_id = "local";

  system_pose_pub_->publish(poses_arr_msg);
  
}


void SystemInterface::localCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  local_x_ = msg.pose.position.x;
  local_y_ = msg.pose.position.y;
}

void SystemInterface::headingCallback(const std_msgs::msg::Float32& msg)
{
  heading_in_ = msg.data;
}

void SystemInterface::internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  internal_lat_ = msg.latitude;
  internal_lon_ = msg.longitude;
  internal_alt_ = msg.altitude;
}


void SystemInterface::externalCasaCallback(const casa_msgs::msg::CasaInterface& msg)
{
  // DEPRACATED

  //rclcpp::Time t = msg.header.stamp;
  RCLCPP_INFO_STREAM(this->get_logger(), "id: "<<temp_id);
  sys_id_in_ = msg.sys_id;
  lat_in_ = msg.latitude;
  lon_in_ = msg.longitude;
  alt_in_ = msg.altitude;

  if (std::count(system_ids_.begin(), system_ids_.end(), sys_id_in_))
    {
      // if the sys_id is in the list, update the data
      system_tracker_.at(sys_id_in_).setLatLon(lat_in_, lon_in_);
      system_tracker_.at(sys_id_in_).setAlt(alt_in_);
      //system_tracker_.at(sys_id_in_).setTime(t);
      system_tracker_.at(sys_id_in_).calcRelativeXY(internal_lat_, internal_lon_, local_x_, local_y_);
    }
  else
    {
      // create a new agent tracker object. 
      AgentTracker agent(sys_id_in_, lat_in_, lon_in_, alt_in_, internal_lat_, internal_lon_, local_x_, local_y_);
      system_ids_.push_back(sys_id_in_);
      system_tracker_.insert(std::pair<int,AgentTracker>(sys_id_in_,agent));
    }

}


