/* Author: Jason Hughes
 * Date: Feb. 2023
 * About: utility functions for CASA
 */

#include "utils/casa_utils.hpp"

Eigen::Vector2d llToUTM(float lat, float lon)
{
  // utility function to convert (lat, lon) coordinates to UTM coordinates
  const float a = 6378137.0; // equatorial radius based on WGS-84
  const float ecc_squared = 0.00669438;
  const float k0 = 0.9996;
  const float deg_to_rad = M_PI/180.0;
  
  float zone_number;
  
  float lon_temp = (lon+180.0) - static_cast<int>((lon+180)/360)*360-180;

  float lat_rad = lat * deg_to_rad;
  float lon_rad = lon_temp * deg_to_rad;
  
  zone_number = static_cast<int>((lon_temp+180)/6)+1.0;
  
  if ( lat >= 56.0 && lat < 64.0 && lon_temp >= 3.0 && lon_temp < 12.0)
    {
      zone_number = 32.0;
    }

  //special zones for Svalbard
  if (lat >= 72.0 && lat < 84.0)
    {
      if (lon_temp >= 0.0 && lon_temp < 9.0) { zone_number = 31; }
      else if (lon_temp >= 9.0 && lon_temp < 21.0) { zone_number = 33; }
      else if (lon_temp >= 21.0 && lon_temp < 33.0) { zone_number = 35; }
      else if (lon_temp >= 33.0 && lon_temp < 42.0) { zone_number = 37; }
    }

  float lon_origin = (zone_number-1.0) * 6.0 -180.0 + 3.0; // +3 puts origin in the middle of the zone
  float lon_origin_rad = lon_origin * deg_to_rad;
  
  double ecc_prime_squared = (ecc_squared)/(1.0-ecc_squared);
  double N = a/sqrt(1.0-ecc_squared * sin(lat_rad)*sin(lat_rad));
  double T = tan(lat_rad) * tan(lat_rad);
  double C = ecc_prime_squared * cos(lat_rad) * cos(lat_rad);
  double A = cos(lat_rad) * (lon_rad - lon_origin_rad);
  
  float M = a * ((1.0 - ecc_squared/4.0
		  - 3.0*ecc_squared*ecc_squared/64.0
		  - 5.0*ecc_squared*ecc_squared*ecc_squared/256.0)*lat_rad
		 -(3.0*ecc_squared/8.0
		   +3.0*ecc_squared*ecc_squared/32.0
		   +45.0*ecc_squared*ecc_squared*ecc_squared/1024.0)*sin(2.0*lat_rad)
		 +(15.0*ecc_squared*ecc_squared/256.0 + 45.0*ecc_squared*ecc_squared*ecc_squared/1024.0)*sin(4.0*lat_rad)
		 -(35.0*ecc_squared*ecc_squared*ecc_squared/3072.0)*sin(6.0*lat_rad));

  
  double utm_easting = (k0*N*(A+(1.0-T+C)*A*A*A/6.0
			     +(5.0-18.0*T+T*T+72.0*C-58.0*ecc_prime_squared)*A*A*A*A*A/120.0)
		       + 500000.0);
  double utm_northing = (k0*(M+N*tan(lat_rad)*(A*A/2.0+(5-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
					      + (61.0-58.0*T+T*T+600.0*C-300.0*ecc_prime_squared)*A*A*A*A*A*A/720.0)));
  if (lat < 0)
    {
      utm_northing = utm_northing + 10000000.0;
    }

  Eigen::Vector2d utm(utm_easting, utm_northing);
  
  return utm;
}


Eigen::Vector2d manhattanDistance(float x1, float y1, float x2, float y2)
{
  Eigen::Vector2d coord1(x1,y1);
  Eigen::Vector2d coord2(x2,y2);

  return coord1 - coord2;
}

int main()
{
  return 0;
}
