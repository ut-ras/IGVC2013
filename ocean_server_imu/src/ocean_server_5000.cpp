/*
* os5000.cpp
*
*  Created on: Mar 10, 2011
*      Author: Nicu Stiurca, Joshua James
*/
#include <sstream>

#include "ocean_server_imu/ocean_server_5000.hpp"
using namespace std;

uint8_t computeChecksum(const string &nmea_sentence)
{
  uint8_t chksum = 0x00;

  assert('$' == nmea_sentence[0]);

  size_t i = 1;
  while('*' != nmea_sentence[i]) 
  {
    chksum ^= nmea_sentence[i++];
  }

  return chksum;
}

bool isValid(const string &nmea_sentence) 
{
  if('$' != nmea_sentence[0]) 
  {
    return false;
  }

  uint8_t calculated_chksum = computeChecksum(nmea_sentence);
  unsigned int expected_chksum;

  size_t chksum_index = nmea_sentence.find('*') + 1;

  sscanf(nmea_sentence.substr(chksum_index).c_str(), "%x", &expected_chksum);

  return calculated_chksum == (uint8_t)expected_chksum;
}

double headingToYaw(double heading)
{
  // fix the heading to fit in the World Coordinate System
  heading -= 90.0; // move origin to +x axis
  if(heading < 0.0)
  {
    heading += 360.0; // wrap to a positive
  }
  if(heading >= 360.0)
  {
    heading -= 360.0;
  }

  heading = 360.0 - heading; 	// counter-clockwise

  if(heading > 180.0)
  {
    heading -= 360.0;
  }

  return dtor(heading);
}

bool parseOSData(const string &nmea_sentence, ocean_server_imu::RawData &msg)
{
  bool success = true;

  char field;
  double value;
  stringstream string_stream(nmea_sentence);

  string_stream >> field;
  assert('$' == field);

  //Sample data string from imu: "$C69.2P0.4R154.0T18.3D-212797.5424M92.41Mx-18.45My-47.07Mz77.35A1.018Ax0.007Ay0.446Az-0.915G-0.01Gx-0.01Gy0.00L553295*6A"
  while(string_stream.peek() != '*') 
  {
    string_stream >> field;
    switch(field) 
    {
    case 'C':
      string_stream >> value;
      msg.yaw = headingToYaw(value);
      break;
    case 'P':
      string_stream >> value;
      msg.pitch = dtor(value);
      break;
    case 'R':
      string_stream >> value;
      msg.roll = dtor(value);
      break;
    case 'T':
      string_stream >> value;
      msg.temperature = value;
      break;
    case 'D':
      string_stream >> value;
      msg.depth = ft2m(value);
      break;
    case 'M':
      switch(string_stream.peek()) 
      {
      case 'x':
        string_stream >> field >> value;
        msg.magnetic.x = value;
        break;
      case 'y':
        string_stream >> field >> value;
        msg.magnetic.y = value;
        break;
      case 'z':
        string_stream >> field >> value;
        msg.magnetic.z = value;
        break;
      default:
        string_stream >> value;
        msg.magnetic_vector_length = value;
        break;
      }
      break;
    case 'A':
      switch(string_stream.peek()) 
      {
      case 'x':
        string_stream >> field >> value;
        msg.acceleration.x = gForceToAccel(value);
        break;
      case 'y':
        string_stream >> field >> value;
        msg.acceleration.y = gForceToAccel(value);
        break;
      case 'z':
        string_stream >> field >> value;
        msg.acceleration.z = gForceToAccel(value);
        break;
      default:
        string_stream >> value;
        msg.acceleration_vector_length = gForceToAccel(value);
        break;
      }
      break;
    case 'G':
      switch(string_stream.peek()) 
      {
      case 'x':
        string_stream >> field >> value;
        msg.gyroscope.x = value;
        break;
      case 'y':
        string_stream >> field >> value;
        msg.gyroscope.y = value;
        break;
      default:
        string_stream >> value;
        msg.gyroscope_vector_length = value;
        break;
      }
      break;
    case 'L':
      string_stream >> msg.header.seq;
      break;
    default:
      ROS_WARN_STREAM_ONCE("Unknown field delimiter: " << field << " in senctence " << nmea_sentence);
//      success = false;
      break;
    }
  }

  return success;
}
