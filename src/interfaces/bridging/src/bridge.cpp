/* Author: Jason Hughes
 * Date: April 2023
 * About: class for the topic bridging. 
 * NOTES:
 * - I'm using Reentrant callback scheduling, meaning ROS will schedule the callbacks for the the timer as well
 *   as the ones created by domain_bridge on a single thread. Since this is at relatively slow pace this shouldn' matter.
 */


#include "bridging/bridge.hpp"

Bridge::Bridge() : Node("bridging")
{
  // Wait for 5 seconds for topics to populate
  RCLCPP_INFO(get_logger(), "Bridge waiting for topics to populate");
  rclcpp::sleep_for(std::chrono::seconds(5));
  
  this -> declare_parameter("sys_id", 1);
  my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
  std::string nspc = "casa"+std::to_string(my_id_);

  rclcpp::QoS qos(10);
  qos.keep_last(10);
  qos.best_effort();
  qos.transient_local();

  group_ = this -> create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  //options_.callback_group -> group_;
  
  sys_id_sub_ = this -> create_subscription<std_msgs::msg::UInt16MultiArray>(nspc + "/internal/system_ids",
									     qos,
									     std::bind(&Bridge::sysIdCallback, this, std::placeholders::_1));

  timer_ = this -> create_wall_timer(1000ms, std::bind(&Bridge::timerCallback, this), group_);  
}


void Bridge::timerCallback()
{
  RCLCPP_DEBUG(get_logger(),"Looking for topics to bridge");
  topic_map_ = get_topic_names_and_types();
  extractTopicsAndBridge();
}


void Bridge::sysIdCallback(const std_msgs::msg::UInt16MultiArray& msg)
{
  std::vector<uint16_t> ids_in = msg.data;

  int num_ids = ids_in.size();
  
  for(int i = 0; i < num_ids; i++)
    {
      if (!(std::count(system_ids_.begin(), system_ids_.end(), ids_in[i])))
	{
	  // if its not an already tracked id, add it to the vector
	  system_ids_.push_back(ids_in[i]);
	}
    }
	    
}


void Bridge::extractTopicsAndBridge()
{
  std::map<std::string, std::vector<std::string>>::iterator it;

  const char del = '/';
  
  for (it = topic_map_.begin(); it != topic_map_.end(); it++)
    {
      std::string full_topic = it -> first;
      std::string type = it -> second[0];
      std::vector<std::string> split_str = splitString(full_topic, del);
      //RCLCPP_INFO_STREAM(get_logger(),"Topic Name: " << it->first);
      for (std::string istr: split_str)
      	{
	  // add a domain bridge if the topic is external and hasn't been bridged yet
      	  if ((istr == "external") && !(bridged_topics_.count(full_topic)))
	    {
	      addBridge(full_topic, type);
	      bridged_topics_.insert(full_topic);
	    }
      	}
    }
}


void Bridge::addBridge(std::string topic, std::string type)
{
  RCLCPP_INFO_STREAM(get_logger(), "bridging: "<<topic);
  domain_bridge::TopicBridgeOptions tbo;
  tbo.callback_group() = group_;
  
  for (int to_id : system_ids_)
    {
      // bride_topic(topic, msg_type, from, to)
      domain_bridge_.bridge_topic(topic, type, my_id_, to_id, tbo);
      RCLCPP_INFO_STREAM(get_logger(), "bridging from " << my_id_ << " to " << to_id << " on topic: " << topic);
    }
  
}
