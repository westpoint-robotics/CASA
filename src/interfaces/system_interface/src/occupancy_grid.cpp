/* Author: Jason Hughes
 * Date: Feb 2023
 * About: Class for occupancy grid building 
 */

#include "system_interface/occupancy_grid.hpp"

OccupancyGrid::OccupancyGrid(float resolution, int width, int height)
{
  setMapResolution(resolution);
  setMapWidth(width);
  setMapHeight(height);
  setGrid(width, height, resolution);
}

void OccupancyGrid::setMapResolution(float resolution)
{
  map_resolution_ = resolution;
}

void OccupancyGrid::setMapWidth(int width)
{
  width_ = width;
}

void OccupancyGrid::setMapHeight(int height)
{
  height_ = height;
}

void OccupancyGrid::setMapOrigin(geometry_msgs::msg::Pose origin)
{
  map_origin_ = origin;
}

void OccupancyGrid::setGrid(int width, int height, float resolution)
{
  grid_ = Eigen::ArrayXXf::Zero(width / static_cast<int>(resolution), height / static_cast<int>(resolution));
}

float OccupancyGrid::getMapResolution()
{
  return map_resolution_;
}

int OccupancyGrid::getMapWidth()
{
  return width_;
}

int OccupancyGrid::getMapHeight()
{
  return height_;
}

Eigen::ArrayXXf OccupancyGrid::getGrid()
{
  return grid_;
}

geometry_msgs::msg::Pose OccupancyGrid::getMapOrigin()
{
  return map_origin_;
}


int main()
{
  return 0;
}

