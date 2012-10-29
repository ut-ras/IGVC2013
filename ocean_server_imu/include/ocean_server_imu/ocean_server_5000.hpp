/*
 * os5000.hpp
 *
 *  Created on: Mar 10, 2011
 *      Author: ras
 */

#ifndef OS5000_HPP_
#define OS5000_HPP_
#include <string>

#include "ocean_server_imu/RawData.h"

/** Convertes degrees to radians. */
inline double dtor(double deg)
{
  return deg * M_PI/180.0;
}

/** Converts feet to meters. */
inline double ft2m(double feet)
{
  return feet * 12.0 * 2.54 * 0.01;
}

/** Converts heading (degrees clockwise from North)
 * to yaw (radians counter-clockwise from +x axis).
 */
double headingToYaw(double heading);

/** Converts g-forces (ie, number of g's)
 * to acceleration (ie, meters per second squared).
 *
 * Conversion factor taken from Wikipedia.
 */
inline double gForceToAccel(double gForce)
{
  return gForce * 9.802665;
}

uint8_t computeChecksum(const std::string &nmea_sentence);
bool isValid(const std::string &nmea_sentence);
bool parseOSData(const std::string &nmea_sentence, ocean_server_imu::RawData &msg);


#endif /* OS5000_HPP_ */
