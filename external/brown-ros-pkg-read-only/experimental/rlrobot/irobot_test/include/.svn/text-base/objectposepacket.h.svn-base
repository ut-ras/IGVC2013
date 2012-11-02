/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef OBJECT_POSE_PACKET_H
#define OBJECT_POSE_PACKET_H OBJECT_POSE_PACKET_H

#include "./packet.h"
#include "./localizableobject.h"
#include <vector>

namespace rlab {

class ObjectPosePacket : public Packet
{
public:
  ObjectPosePacket();
  virtual ~ObjectPosePacket();

  static ObjectPosePacket* parse(const std::string& packet);
  bool generate(std::string& packet);

  void reset();

  void addLocalizableObject(const LocalizableObject& obj);
  void addLocalizableObjects(const std::vector<LocalizableObject>& obj);
  unsigned int getLocalizableObjectCount() const;
  const LocalizableObject& getLocalizableObject(unsigned int idx) const;

protected:
  std::vector<LocalizableObject> objects_;
};

} // namespace rlab

#endif // OBJECT_POSE_PACKET_H

