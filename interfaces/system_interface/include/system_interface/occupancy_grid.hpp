/* Author: Jason Hughes
 * Date:   Feb. 2023
 * About:  
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <map>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "utils/casa_utils.hpp"
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
  void setGrid(int width, int height, float resolution);

  
  float getMapResolution();
  int getMapWidth();
  int getMapHeight();

  Eigen::ArrayXXf getGrid();
  geometry_msgs::msg::Pose getMapOrigin();
  
private:

  geometry_msgs::msg::Pose map_origin_;
  Eigen::ArrayXXf grid_;
  float map_resolution_;
  int width_, height_;
  
};
#endif
