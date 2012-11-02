/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef LOCALIZABLE_OBJECT_H
#define LOCALIZABLE_OBJECT_H LOCALIZABLE_OBJECT_H

#include "./objectpose.h"

namespace rlab {

class LocalizableObject
{
public:
  enum Type {
    OBJECT_TYPE_SMURV,
    OBJECT_TYPE_BALL,
    OBJECT_TYPE_OBSTACLE,
    OBJECT_TYPE_UNKNOWN
  };

public:
  LocalizableObject();
  virtual ~LocalizableObject();

  void setType(LocalizableObject::Type type);
  LocalizableObject::Type getType() const;

  void setId(unsigned int id);
  unsigned int getId() const;

  void setPose(const ObjectPose& pose);
  const ObjectPose& getPose() const;

  void setPoseAge(unsigned int age);
  unsigned int getPoseAge() const;

protected:
  LocalizableObject::Type type_;
  unsigned int id_;
  ObjectPose pose_;
  unsigned int pose_age_;
};

} // namespace rlab

#endif // LOCALIZABLE_OBJECT_H

