/* Author: Jason Hughes
 * Date:   Feb. 2023
 * About: Header file   
 */

#ifndef AGENT_TRACKER_H
#define AGENT_TRACKER_H

#include <eigen3/Eigen/Dense>

#include "utils/casa_utils.hpp"

class AgentTracker
{
public:

  //constructor
  AgentTracker( int sys_id, float lat, float lon, float alt, float int_lat, float int_lon, float int_x, float int_y );
  
  //setters
  void setSysId( int sys_id );
  void setLatLon( float lat, float lon );
  void setAlt( float alt );
  void setRelativeXY( float x, float y);
  void setTask( int t, float lat, float lon );
  
  //getters
  int getSysId();
  Eigen::Vector2d getLatLon();
  float getAlt();
  Eigen::Vector2d getRelativeXY();
  int getTaskIter();

  void calcRelativeXY(float internal_lat, float internal_lon, float internal_x, float internal_y);
  
private:
  //class variables
  int sys_id_;
  float lat_;
  float lon_;
  float alt_;
  float relative_x_;
  float relative_y_;
  int task_iter_;
  float task_lat_;
  float task_lon_;
};
#endif

