/* Author: Jason Hughes
 * Date: July 2023
 * About: trajectory planning for behaviors
 */

#ifndef BEHAVIOR_TRAJECTORY_HPP
#define BEHAVIOR_TRAJECTORY_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;


class BehaviorTrajectory : public rclcpp::Node
{
public:
  BehaviorTrajectory();

private:

  
};
#endif
