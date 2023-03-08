/* Author: Jason Hughes
 * Date: Feb. 2023
 * About:
 */

#ifndef CASA_UTILS_H
#define CASA_UTILS_H

#include <cmath>
#include <eigen3/Eigen/Dense>

Eigen::Vector2d llToUTM(float lat, float lon);
Eigen::Vector2d manhattanDistance(float x1, float y1, float x2, float y2);
float intToFloatConversion(int input, int flag);
			   
#endif
