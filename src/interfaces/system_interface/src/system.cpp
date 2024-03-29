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

  iterator_ = 0;
  
  this -> declare_parameter("sys_id", 1);
  this -> declare_parameter("dropout", 30.0);
  this -> declare_parameter("use_sim", false);
  this -> declare_parameter("system_ids", system_ids_);
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  dropout_ = this -> get_parameter("dropout").get_parameter_value().get<float>();
  use_sim_ = this -> get_parameter("use_sim").get_parameter_value().get<bool>();
  system_ids_ = this -> get_parameter("system_ids").get_parameter_value().get<std::vector<long int>>();
  
  if (use_sim_ == true)
  {
    this -> declare_parameter("num_agents", 2);
    num_agents_ = this -> get_parameter("num_agents").get_parameter_value().get<int>();
  }
  
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

  task_sub_ = this -> create_subscription<std_msgs::msg::Int32>(namespace_+"/internal/task_number",
								qos,
								std::bind(&SystemInterface::myTaskCallback, this, std::placeholders::_1));
  //need the number of agents in the swarm 
  for (int i = 0; i <= num_agents_; i++)
    {
      long int ag = system_ids_[i];
      std::string casa_topic = "casa"+std::to_string(ag)+"/external/exchange";
	
      if (ag != my_id_)
	{
	  RCLCPP_INFO_STREAM(this->get_logger(), "agent: " << my_id_ << " subscribing to the external topic of agent: "<< ag);
	  auto casa_sub = this -> create_subscription<casa_msgs::msg::CasaInterface>(casa_topic,
	  									     qos,
	  									     std::bind(&SystemInterface::externalCasaCallback ,this, std::placeholders::_1));
	  casa_references_.push_back(casa_sub);
	}
    }

  system_pose_pub_ = this -> create_publisher<casa_msgs::msg::CasaAgentArray>(namespace_+"/internal/system_array", qos);
  external_pub_ = this -> create_publisher<casa_msgs::msg::CasaInterface>(namespace_+"/external/exchange", qos);
  system_task_pub_ = this -> create_publisher<casa_msgs::msg::CasaTaskArray>(namespace_+"/internal/system_tasks", qos);
  timer_ = this -> create_wall_timer(100ms, std::bind(&SystemInterface::timerCallback, this));
}


void SystemInterface::timerCallback()
{
  posePublisher();
  exchangePublisher();
  taskPublisher();

  if (dropout_ != 0)
    {
      checkTime();
    }
  
}


void SystemInterface::checkTime()
{
  // function to remove elements from the system_tracker if you have not heard from them
  // std::map<int,AgentTracker>::iterator iter;
  // rclcpp::Time now = this -> get_clock() -> now();
  
  // for (iter = system_tracker_.begin(); iter != system_tracker_.end(); ++iter)
  //   {
  //     rclcpp::Time last_seen = iter->second.getTime();
      
  //     if ( now.seconds() - last_seen.seconds() > dropout_ )
  // 	{
  // 	  system_tracker_.erase(iter);
  // 	}
  //  }
}

void SystemInterface::myTaskCallback(const std_msgs::msg::Int32& msg)
{
  my_task_ = msg.data;
}


void SystemInterface::exchangePublisher()
{
  casa_msgs::msg::CasaInterface msg;

  msg.sys_id = my_id_;
  
  msg.latitude = internal_lat_;
  msg.longitude = internal_lon_;
  msg.altitude = internal_alt_;

  msg.heading = internal_heading_;

  msg.assigned_task = my_task_;
  msg.battery = 25.02;
  msg.ekf_healthy = true;
  msg.connectivity_level = conn_level_;
  
  external_pub_ -> publish(msg);
}


void SystemInterface::posePublisher()
{
  casa_msgs::msg::CasaAgentArray agent_arr_msg;
  std::vector<casa_msgs::msg::CasaAgent> agent_vec;

  for(unsigned int i = 0; i < system_tracker_.size(); i++)
    {
      casa_msgs::msg::CasaAgent agent_reference_pose;

      agent_reference_pose.global_pose.latitude = system_tracker_[i].getLat();
      agent_reference_pose.global_pose.longitude = system_tracker_[i].getLon();
      agent_reference_pose.global_pose.altitude = system_tracker_[i].getAlt();

      agent_reference_pose.global_pose.easting = system_tracker_[i].getEasting();
      agent_reference_pose.global_pose.northing = system_tracker_[i].getNorthing();
      
      agent_reference_pose.local_pose.x = system_tracker_[i].getRelativeXY()[0];
      agent_reference_pose.local_pose.y = system_tracker_[i].getRelativeXY()[1];
      agent_reference_pose.local_pose.z = system_tracker_[i].getAlt();

      agent_reference_pose.assigned_task = system_tracker_[i].getTaskIter();
      agent_reference_pose.connectivity_level = system_tracker_[i].getConnectivityLevel();
      
      agent_reference_pose.sys_id = system_tracker_[i].getSysId();
      agent_reference_pose.heading = system_tracker_[i].getHeading();
      
      agent_vec.push_back(agent_reference_pose);
    }

  agent_arr_msg.agents = agent_vec;
  agent_arr_msg.header.stamp = this -> get_clock() -> now();
  agent_arr_msg.header.frame_id = "all";

  system_pose_pub_->publish(agent_arr_msg);
  
}

void SystemInterface::taskPublisher()
{
  casa_msgs::msg::CasaTaskArray task_arr_msg;
  std::vector<casa_msgs::msg::CasaTask> sys_tasks;
  
  for(unsigned int i = 0; i < system_tracker_.size(); i++)
    {
      casa_msgs::msg::CasaTask task_msg;
      task_msg.sys_id = system_tracker_[i].getSysId();
      task_msg.assigned_task = system_tracker_[i].getTaskIter();

      sys_tasks.push_back(task_msg);
    }

  task_arr_msg.system_tasks = sys_tasks;
  task_arr_msg.header.stamp = this -> get_clock() -> now();
  task_arr_msg.header.frame_id = "tasks";

  system_task_pub_->publish(task_arr_msg);
}


void SystemInterface::localCallback(const geometry_msgs::msg::PoseStamped& msg)
{
  local_x_ = msg.pose.position.x;
  local_y_ = msg.pose.position.y;
}

void SystemInterface::headingCallback(const std_msgs::msg::Float32& msg)
{
  internal_heading_ = msg.data;
}

void SystemInterface::internalGpsCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  internal_lat_ = msg.latitude;
  internal_lon_ = msg.longitude;
  internal_alt_ = msg.altitude;
}


void SystemInterface::externalCasaCallback(const casa_msgs::msg::CasaInterface& msg)
{
  sys_id_in_ = msg.sys_id;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Agent: " << my_id_ << " talking to agent: " << sys_id_in_);
  lat_in_ = msg.latitude;
  lon_in_ = msg.longitude;
  alt_in_ = msg.altitude;
  heading_in_ = msg.heading;
  task_in_ = msg.assigned_task;
  level_in_ = msg.connectivity_level;

  if (std::count(system_ids_.begin(), system_ids_.end(), sys_id_in_))
    {
      // if the sys_id is in the list, update the data
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Updating agent: " << sys_id_in_);
      int iter = iter_tracker_.at(sys_id_in_);
      system_tracker_[iter].setLatLon(lat_in_, lon_in_);
      system_tracker_[iter].setAlt(alt_in_);
      //system_tracker_[iter].setTime(t);
      system_tracker_[iter].setHeading(heading_in_);
      system_tracker_[iter].setTask(task_in_);
      system_tracker_[iter].calcAndSetUTM(lat_in_, lon_in_);
      system_tracker_[iter].calcRelativeXY(internal_lat_, internal_lon_, local_x_, local_y_);
    }
  else
    {
      //create a new agent tracker object.
      RCLCPP_INFO_STREAM(this->get_logger(), "Creating tracker for Agent: " << sys_id_in_);

      AgentTracker agent(sys_id_in_, lat_in_, lon_in_, alt_in_, internal_lat_, internal_lon_, local_x_, local_y_, heading_in_, task_in_, level_in_);

      system_ids_.push_back(sys_id_in_);
      system_tracker_.push_back(agent);

      iter_tracker_.insert({sys_id_in_, iterator_});
      iterator_++;
    }
}


