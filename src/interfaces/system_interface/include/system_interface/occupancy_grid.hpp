/* Author: Jason Hughes
 * Date:   Feb. 2023
 * About:  
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <map>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

class OccupancyGrid
{
public:
  OccupancyGrid(float resolution, int width, int height);

  void setMapResolution( float resolution );
  void setMapWidth( int width );
  void setMapHeight( int height );
  void setMapOrigin( geometry_msgs::msg::Pose origin );
  
  float getMapResolution();
  int getMapWidth();
  int getMapHeight();
  geometry_msgs::msg::Pose getMapOrigin();
  
  nav_msgs::msg::OccupancyGrid buildGrid( std::map<int, float(*)[3]> input_map, geometry_msgs::msg::Pose current_pose );
  
private:

  Eigen::Vector2d globalToLocal(float external_lat, float external_lon, float internal_x, float internaly, float internal_lat, float internal_lon);

  Eigen::Vector2d llToUTM(float lat, float lon);
  
  nav_msgs::msg::OccupancyGrid ros_ogrid_;
  geometry_msgs::msg::Pose map_origin_;
  
  float map_resolution_;
  int width_, height_;
  
};
#endif
