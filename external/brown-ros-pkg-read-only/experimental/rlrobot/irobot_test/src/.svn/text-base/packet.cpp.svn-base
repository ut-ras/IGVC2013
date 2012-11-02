/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#include "../include/packet.h"
#include <netinet/in.h>
#include <iostream>

namespace rlab {

using namespace std;

//-----------------------------------------------------------------------------------------------------------
Packet::Packet()
{
}


//-----------------------------------------------------------------------------------------------------------
Packet::~Packet()
{
}


//-----------------------------------------------------------------------------------------------------------
Packet* Packet::parse(const string& packet)
{
  // To be implemented by subclasses, that is specific packet types
  return NULL;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::generate(string& packet)
{
  // To be implemented by subclasses, that is specific packet types
  return false;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::parseUInt8(const string& packet, unsigned int& offset, unsigned char& value)
{
  if (packet.length() < offset + 1) {
    // Packet not long enough
    return false;
  }

  value = packet[offset];
  ++offset;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::parseUInt16(const string& packet, unsigned int& offset, unsigned int& value)
{
  if (packet.length() < offset + 2) {
    // Packet not long enough
    return false;
  }

  const char* p = packet.c_str() + offset;
  value = (((unsigned int)p[0] << 8) & 0xFF00)
      | ((unsigned int)p[1] & 0x00FF);
  value = ntohs(value);
  offset += 2;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::parseUInt32(const string& packet, unsigned int& offset, unsigned int& value)
{
  if (packet.length() < offset + 4) {
    // Packet not long enough
    return false;
  }

  const char* p = packet.c_str() + offset;
  value = (((unsigned int)p[0] << 24) & 0xFF000000)
      | (((unsigned int)p[1] << 16) & 0x00FF0000)
      | (((unsigned int)p[2] << 8) & 0x0000FF00)
      | ((unsigned int)p[3] & 0x000000FF);
  value = ntohl(value);
  offset += 4;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::parseFloat32(const string& packet, unsigned int& offset, float& value)
{
  unsigned int v;
  if (!parseUInt32(packet, offset, v)) {
    return false;
  }

  value = *reinterpret_cast<float*>(&v);

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::generateUInt8(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned char value)
{
  if (bufferLength < offset + 1) {
    // Buffer not big enough
    return false;
  }

  buffer[offset] = value;
  ++offset;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::generateUInt16(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned int value)
{
  if (bufferLength < offset + 2) {
    // Buffer not big enough
    return false;
  }

  value = htons(value);
  buffer[offset] = (char)((value >> 8) & 0x00FF);
  buffer[offset + 1] = (char)(value & 0x00FF);
  offset += 2;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::generateUInt32(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned int value)
{
  if (bufferLength < offset + 4) {
    // Buffer not big enough
    return false;
  }

  value = htonl(value);
  buffer[offset] = (char)((value >> 24) & 0x000000FF);
  buffer[offset + 1] = (char)((value >> 16) & 0x000000FF);
  buffer[offset + 2] = (char)((value >> 8) & 0x000000FF);
  buffer[offset + 3] = (char)(value & 0x000000FF);
  offset += 4;

  return true;
}


//-----------------------------------------------------------------------------------------------------------
bool Packet::generateFloat32(char* buffer, unsigned int& offset, unsigned int bufferLength, float value)
{
  unsigned int v = *reinterpret_cast<unsigned int*>(&value);
  return generateUInt32(buffer, offset, bufferLength, v);
}

} // namespace rlab

