/* Author: Jason Hughes
 * Date: Feb. 2023
 * About: class for tracking the Multi-agent system
 */

#include "system_interface/agent_tracker.hpp"

AgentTracker::AgentTracker(int sys_id, float lat, float lon, float alt, float int_lat, float int_lon, float int_x, float int_y, float heading)
{
  setSysId(sys_id);
  setLatLon(lat, lon);
  setAlt(alt);
  setHeading(heading);

  calcAndSetUTM(lat, lon);
  calcRelativeXY(int_lat, int_lon, int_x, int_y);
}

AgentTracker::AgentTracker(const AgentTracker &a)
{
  sys_id_ = a.sys_id_;
  lat_ = a.lat_;
  lon_ = a.lon_;
  alt_ = a.alt_;
  relative_x_ = a.relative_x_;
  relative_y_ = a.relative_y_;
  time_ = a.time_;
  easting_ = a.easting_;
  northing_ = a.northing_;
  heading_ = a.heading_;
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

void AgentTracker::setHeading(float h)
{
  heading_ = h;
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

void AgentTracker::setEasting(float e)
{
  easting_ = e;
}

void AgentTracker::setNorthing(float n)
{
  northing_ = n;
}

void AgentTracker::setEastingNorthing(Eigen::Vector2d utm)
{
  easting_ = utm[0];
  northing_ = utm[1];
}

int AgentTracker::getSysId()
{
  return sys_id_;
}

float AgentTracker::getHeading()
{
  return heading_;
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

float AgentTracker::getLat()
{
  return lat_;
}

float AgentTracker::getLon()
{
  return lon_;
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

float AgentTracker::getEasting()
{
  return easting_;
}

float AgentTracker::getNorthing()
{
  return northing_;
}

Eigen::Vector2d AgentTracker::getEastingNorthing()
{
  Eigen::Vector2d en(easting_, northing_);
  return en;
}

Eigen::Vector2d AgentTracker::calcEastingNorthing(float lat, float lon)
{
  Eigen::Vector2d utm = llToUTM(lat, lon);
  return utm;
}

void AgentTracker::calcAndSetUTM(float lat, float lon)
{
  Eigen::Vector2d utm = llToUTM(lat, lon);
  setEastingNorthing(utm);
}

void AgentTracker::calcRelativeXY(float internal_lat, float internal_lon, float internal_x,float internal_y)
{
  Eigen::Vector2d external_utm = llToUTM(lat_, lon_);
  Eigen::Vector2d internal_utm = llToUTM(internal_lat, internal_lon);

  Eigen::Vector2d diff = internal_utm - external_utm;

  setRelativeXY( internal_x + diff[0], internal_y + diff[1] );
}
