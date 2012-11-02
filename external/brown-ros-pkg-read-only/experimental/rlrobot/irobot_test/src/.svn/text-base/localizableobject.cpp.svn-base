/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#include "../include/localizableobject.h"

namespace rlab {

using namespace std;

//-----------------------------------------------------------------------------------------------------------
LocalizableObject::LocalizableObject()
  : type_(OBJECT_TYPE_UNKNOWN),
    id_(0),
    pose_age_(0)
{
  pose_.setX(0.0);
  pose_.setY(0.0);
  pose_.setYaw(0.0);
}


//-----------------------------------------------------------------------------------------------------------
LocalizableObject::~LocalizableObject()
{
}


//-----------------------------------------------------------------------------------------------------------
void LocalizableObject::setType(LocalizableObject::Type type) 
{
  type_ = type;
}


//-----------------------------------------------------------------------------------------------------------
LocalizableObject::Type LocalizableObject::getType() const
{
  return type_;
}


//-----------------------------------------------------------------------------------------------------------
void LocalizableObject::setId(unsigned int id)
{
  id_ = id;
}


//-----------------------------------------------------------------------------------------------------------
unsigned int LocalizableObject::getId() const
{
  return id_;
}


//-----------------------------------------------------------------------------------------------------------
void LocalizableObject::setPose(const ObjectPose& pose)
{
  pose_ = pose;
}


//-----------------------------------------------------------------------------------------------------------
const ObjectPose& LocalizableObject::getPose() const
{
  return pose_;
}


//-----------------------------------------------------------------------------------------------------------
void LocalizableObject::setPoseAge(unsigned int age)
{
  pose_age_ = age;
}


//-----------------------------------------------------------------------------------------------------------
unsigned int LocalizableObject::getPoseAge() const
{
  return pose_age_;
}

} // namespace rlab

