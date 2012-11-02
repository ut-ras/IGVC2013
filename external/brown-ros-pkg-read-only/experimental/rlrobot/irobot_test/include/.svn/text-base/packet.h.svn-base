/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef PACKET_H
#define PACKET_H PACKET_H

#include <string>

namespace rlab {

class Packet
{
public:
  Packet();
  virtual ~Packet();

  static Packet* parse(const std::string& packet);
  bool generate(std::string& packet);

protected:
  static bool parseUInt8(const std::string& packet, unsigned int& offset, unsigned char& value);
  static bool parseUInt16(const std::string& packet, unsigned int& offset, unsigned int& value);
  static bool parseUInt32(const std::string& packet, unsigned int& offset, unsigned int& value);
  static bool parseFloat32(const std::string& packet, unsigned int& offset, float& value);

  static bool generateUInt8(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned char value);
  static bool generateUInt16(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned int value);
  static bool generateUInt32(char* buffer, unsigned int& offset, unsigned int bufferLength, unsigned int value);
  static bool generateFloat32(char* buffer, unsigned int& offset, unsigned int bufferLength, float value);
};

} // namespace rlab

#endif // PACKET_H

