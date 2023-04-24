/* Author: Jason Hughes
 * Date: Feb. 2023
 * About: class for tracking the Multi-agent system
 */

#include "system_interface/agent_tracker.hpp"

AgentTracker::AgentTracker(int sys_id, float lat, float lon, float alt, float int_lat, float int_lon, float int_x, float int_y)
{
  setSysId(sys_id);
  setLatLon(lat, lon);
  setAlt(alt);

  calcRelativeXY(int_lat, int_lon, int_x, int_y);
}

void AgentTracker::setSysId(int sys_id)
{
  sys_id_ = sys_id;
}

void AgentTracker::setLatLon(float lat, float lon)
{
  lat_ = lat;
  lon_ = lon;
}

void AgentTracker::setAlt(float alt)
{
  alt_ = alt;
}

void AgentTracker::setRelativeXY(float x, float y)
{
  relative_x_ = x;
  relative_y_ = y;
}

void AgentTracker::setTime(rclcpp::Time t)
{
  time_ = t;
}

int AgentTracker::getSysId()
{
  return sys_id_;
}

Eigen::Vector2d AgentTracker::getLatLon()
{
  Eigen::Vector2d ll(lat_, lon_);
  return ll;
}

float AgentTracker::getAlt()
{
  return alt_;
}

Eigen::Vector2d AgentTracker::getRelativeXY()
{
  Eigen::Vector2d xy(relative_x_, relative_y_);
  return xy;
}

rclcpp::Time AgentTracker::getTime()
{
  return time_;
}

void AgentTracker::calcRelativeXY(float internal_lat, float internal_lon, float internal_x,float internal_y)
{
  Eigen::Vector2d external_utm = llToUTM(lat_, lon_);
  Eigen::Vector2d internal_utm = llToUTM(internal_lat, internal_lon);

  Eigen::Vector2d diff = internal_utm - external_utm;

  setRelativeXY( internal_x + diff[0], internal_y + diff[1] );
}
