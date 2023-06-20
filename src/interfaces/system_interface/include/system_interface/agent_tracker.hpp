/* Author: Jason Hughes
 * Date:   Feb. 2023
 * About: Header file   
 */

#ifndef AGENT_TRACKER_H
#define AGENT_TRACKER_H

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "utils/casa_utils.hpp"

class AgentTracker
{
public:

  //constructor
  AgentTracker( int sys_id, float lat, float lon, float alt, float int_lat, float int_lon, float int_x, float int_y, float heading, int task);
  //copy constructor
  AgentTracker(const AgentTracker &a );
  
  //setters
  void setSysId( int sys_id );
  void setLatLon( float lat, float lon );
  void setAlt( float alt );
  void setRelativeXY( float x, float y );
  void setTime( rclcpp::Time t );
  void setEasting( float e );
  void setNorthing( float n );
  void setEastingNorthing( Eigen::Vector2d utm );
  void setHeading( float h );
  void setTask( int t );
  
  //getters
  int getSysId();
  float getLat();
  float getLon();
  Eigen::Vector2d getLatLon();
  float getAlt();
  Eigen::Vector2d getRelativeXY();
  rclcpp::Time getTime();
  float getEasting();
  float getNorthing();
  float getHeading();
  Eigen::Vector2d getEastingNorthing();
  int getTaskIter();
  Eigen::Vector2d getTaskLatLon();
  
  Eigen::Vector2d calcEastingNorthing(float lat, float lon);
  void calcAndSetUTM(float lat, float lon);
  void calcRelativeXY(float internal_lat, float internal_lon, float internal_x, float internal_y);
  
private:
  //class variables
  int sys_id_;
  float lat_;
  float lon_;
  float alt_;
  float relative_x_;
  float relative_y_;
  float easting_;
  float northing_;
  float heading_;
  int task_;
  float task_lat_;
  float task_lon_;
  rclcpp::Time time_;
};
#endif

