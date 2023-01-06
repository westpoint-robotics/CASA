/*!
 * Author: Jason Hughes
 * Date:   Jan. 2023
 * About:  ROS2 node to interface with the rest of the swarm. 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#inlcude "px4_msgs/msg/vehicle_status.hpp"

#include "system_interface/system.hpp"

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface() : Node("system_interface")
