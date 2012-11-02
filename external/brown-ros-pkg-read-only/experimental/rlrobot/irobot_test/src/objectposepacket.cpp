/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#include "../include/objectposepacket.h"
#include "../include/datagram.h"
#include "../include/objectpose.h"
#include <assert.h>

namespace rlab {

using namespace std;

//-----------------------------------------------------------------------------------------------------------
ObjectPosePacket::ObjectPosePacket()
  : Packet(),
    objects_()
{
}


//-----------------------------------------------------------------------------------------------------------
ObjectPosePacket::~ObjectPosePacket()
{
}


//-----------------------------------------------------------------------------------------------------------
ObjectPosePacket* ObjectPosePacket::parse(const string& packetData)
{
  ObjectPosePacket* packet = new ObjectPosePacket();

  unsigned int offset = 0;

  // Read number of objects to expect in this packet
  unsigned char numObjects;
  if (!parseUInt8(packetData, offset, numObjects)) {
    delete packet;
    return NULL;
  }

  // Read each object
  for (unsigned int i = 0; i < numObjects; ++i) {
    unsigned char type;
    if (!parseUInt8(packetData, offset, type)) {
      delete packet;
      return NULL;
    }

    unsigned char id;
    if (!parseUInt8(packetData, offset, id)) {
      delete packet;
      return NULL;
    }

    unsigned int age;
    if (!parseUInt32(packetData, offset, age)) {
      delete packet;
      return NULL;
    }

    float x;
    if (!parseFloat32(packetData, offset, x)) {
      delete packet;
      return NULL;
    }

    float y;
    if (!parseFloat32(packetData, offset, y)) {
      delete packet;
      return NULL;
    }

    float yaw;
    if (!parseFloat32(packetData, offset, yaw)) {
      delete packet;
      return NULL;
    }

    // Create object from data read in
    LocalizableObject obj;
    switch (type) {
      case 1:
        obj.setType(LocalizableObject::OBJECT_TYPE_SMURV);
        break;

      case 2:
        obj.setType(LocalizableObject::OBJECT_TYPE_BALL);
        break;

      case 3:
        obj.setType(LocalizableObject::OBJECT_TYPE_OBSTACLE);
        break;

      default:
        obj.setType(LocalizableObject::OBJECT_TYPE_UNKNOWN);
        break;
    }

    obj.setId(id);
    obj.setPose(ObjectPose(x, y, yaw));
    obj.setPoseAge(age);

    packet->objects_.push_back(obj); 
  }

  return packet;
}


//-----------------------------------------------------------------------------------------------------------
bool ObjectPosePacket::generate(string& packet)
{
  char buffer[Datagram::MAX_LEN + 1];
  unsigned int offset = 0;
  
  // Write out number of objects the packet will contain
  if (!generateUInt8(buffer, offset, Datagram::MAX_LEN, objects_.size())) {
    return false;
  }

  // Write each object
  for (unsigned int i = 0; i < objects_.size(); ++i) {
    unsigned char type;
    switch (objects_[i].getType()) {
      case LocalizableObject::OBJECT_TYPE_SMURV:
        type = 1;
        break;

      case LocalizableObject::OBJECT_TYPE_BALL:
        type = 2;
        break;

      case LocalizableObject::OBJECT_TYPE_OBSTACLE:
        type = 3;
        break;

      case LocalizableObject::OBJECT_TYPE_UNKNOWN:
      default:
        type = 0;
        break;
    }

    if (!generateUInt8(buffer, offset, Datagram::MAX_LEN, type)) {
      return false;
    }

    if (!generateUInt8(buffer, offset, Datagram::MAX_LEN, objects_[i].getId())) {
      return false;
    }

    if (!generateUInt32(buffer, offset, Datagram::MAX_LEN, objects_[i].getPoseAge())) {
      return false;
    }

    ObjectPose pose = objects_[i].getPose();
    if (!generateFloat32(buffer, offset, Datagram::MAX_LEN, pose.getX())) {
      return false;
    }
    if (!generateFloat32(buffer, offset, Datagram::MAX_LEN, pose.getY())) {
      return false;
    }
    if (!generateFloat32(buffer, offset, Datagram::MAX_LEN, pose.getYaw())) {
      return false;
    }
  }

  packet.clear();
  packet.reserve(offset);
  for (unsigned int i = 0; i < offset; ++i) {
    packet += buffer[i];
  }

  return true;
}


//-----------------------------------------------------------------------------------------------------------
void ObjectPosePacket::reset()
{
  objects_.clear();
}


//-----------------------------------------------------------------------------------------------------------
void ObjectPosePacket::addLocalizableObject(const LocalizableObject& obj)
{
  objects_.push_back(obj);
}


//-----------------------------------------------------------------------------------------------------------
void ObjectPosePacket::addLocalizableObjects(const vector<LocalizableObject>& obj)
{
  objects_ = obj;
}


//-----------------------------------------------------------------------------------------------------------
unsigned int ObjectPosePacket::getLocalizableObjectCount() const
{
  return objects_.size();
}


//-----------------------------------------------------------------------------------------------------------
const LocalizableObject& ObjectPosePacket::getLocalizableObject(unsigned int idx) const
{
  assert(idx < objects_.size());
  return objects_[idx];
}

} // namespace rlab

